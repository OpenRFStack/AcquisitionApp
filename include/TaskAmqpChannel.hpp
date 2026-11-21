#pragma once
/*
========================================================================
Project: OpenRFStack
Author:  Brendan Michaud
Year:    2026
Part of OpenRFStack (https://github.com/OpenRFStack)

Licensed under the Personal Use License.
Do not use for commercial, organizational, or military purposes.
Contact author for permission: https://github.com/OpenRFStack
========================================================================
*/
#include <nlohmann/json.hpp>
#include <proton/connection.hpp>
#include <proton/connection_options.hpp>
#include <proton/container.hpp>
#include <proton/delivery.hpp>
#include <proton/message.hpp>
#include <proton/messaging_handler.hpp>
#include <proton/receiver.hpp>
#include <proton/receiver_options.hpp>
#include <proton/reconnect_options.hpp>
#include <proton/sender.hpp>
#include <proton/sender_options.hpp>
#include <proton/source_options.hpp>
#include <proton/symbol.hpp>
#include <proton/target_options.hpp>
#include <proton/transport.hpp>
#include <proton/work_queue.hpp>
#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace acq
{
// ── Persistent AMQP task channel ─────────────────────────────────────────────
// Maintains one long-lived connection to the broker.  The ~30 s Artemis
// subscription-settlement cost is paid ONCE when the channel first connects.
// All subsequent exchange() calls return in < 1 s.
//
// Thread model: proton runs on an internal background thread.  exchange() and
// send() are called from the sweepLoop thread; they post work to the proton
// thread via work_queue and then block on a condition variable.

struct AmqpResponse
{
    bool        received{false};
    std::string body;
};

class TaskAmqpChannel : public proton::messaging_handler
{
public:
    TaskAmqpChannel(std::string url,
                    std::string user,
                    std::string pass,
                    std::string req_queue,
                    std::string resp_queue)
        : url_(std::move(url))
        , user_(std::move(user))
        , pass_(std::move(pass))
        , req_queue_(std::move(req_queue))
        , resp_queue_(std::move(resp_queue))
    {
    }

    ~TaskAmqpChannel()
    {
        stop();
    }

    // Start the background thread and block until sender + receiver are open.
    void start(int connect_timeout_ms = 60'000)
    {
        container_ = std::make_unique<proton::container>(*this);
        thread_    = std::thread(
            [this]
            {
                container_->run();
            });
        std::unique_lock<std::mutex> lk(mu_);
        ready_cv_.wait_for(lk,
                           std::chrono::milliseconds(connect_timeout_ms),
                           [this]
                           {
                               return ready_ || stopped_;
                           });
        if (!ready_)
            spdlog::warn("[TaskAmqpChannel] not ready after {}ms", connect_timeout_ms);
    }

    void stop()
    {
        if (container_)
        {
            {
                // A TASK_STOP held during a reconnect: give the reconnect
                // (2 s retry delay) a bounded chance to deliver it.
                std::unique_lock<std::mutex> lk(mu_);
                if (!held_.empty()
                    && !ready_cv_.wait_for(lk,
                                           std::chrono::milliseconds(STOP_GRACE_MS),
                                           [this]
                                           {
                                               return held_.empty();
                                           }))
                    spdlog::warn("[TaskAmqpChannel] shutting down with {} undelivered message(s)",
                                 held_.size());
            }
            if (auto* wq = wq_.load())
                wq->add(
                    [this]
                    {
                        sender_.connection().close();
                    });
            else
                // Connection never reached on_receiver_open (e.g. Artemis was
                // still starting), so there's no work queue to post a close
                // through. reconnect_options has max_attempts(0) — infinite
                // retries — so without this, container_->run() never returns
                // and thread_.join() below blocks forever.
                container_->stop();
            if (thread_.joinable())
                thread_.join();
            wq_.store(nullptr);
            container_.reset();
        }
    }

    // Send msg_body and wait for a response matching corr_id.
    AmqpResponse
    exchange(const std::string& msg_body, const std::string& corr_id, int timeout_ms = 30'000)
    {
        {
            std::unique_lock<std::mutex> lk(mu_);
            if (!ready_cv_.wait_for(lk,
                                    std::chrono::milliseconds(timeout_ms),
                                    [this]
                                    {
                                        return ready_;
                                    }))
            {
                spdlog::warn("[TaskAmqpChannel] not ready for exchange");
                return {};
            }
            pending_corr_   = corr_id;
            pending_result_ = {};
        }
        if (auto* wq = wq_.load())
            wq->add(
                [this, msg_body]() mutable
                {
                    proton::message msg;
                    msg.body(msg_body);
                    msg.content_type("application/json");
                    msg.reply_to(reply_addr_);
                    if (sender_)
                        sender_.send(msg);
                });
        std::unique_lock<std::mutex> lk(mu_);
        result_cv_.wait_for(lk,
                            std::chrono::milliseconds(timeout_ms),
                            [this]
                            {
                                return pending_result_.received;
                            });
        return pending_result_;
    }

    // Fire-and-forget send (for TASK_STOP). While the channel is down (broker
    // restart, reconnect in progress) the message is held and sent when the
    // links are back: dropping a TASK_STOP left the task, and its radio, with
    // the RM until the task duration ran out.
    void send(const std::string& msg_body)
    {
        {
            std::unique_lock<std::mutex> lk(mu_);
            if (!ready_ || !wq_.load())
            {
                if (held_.size() >= MAX_HELD)
                    held_.erase(held_.begin());  // keep the newest
                held_.push_back(msg_body);
                spdlog::warn("[TaskAmqpChannel] not connected; holding message ({} held)",
                             held_.size());
                return;
            }
        }
        wq_.load()->add(
            [this, msg_body]
            {
                sendNow(msg_body);
            });
    }

    // proton callbacks ─────────────────────────────────────────────────────────
    void on_container_start(proton::container& c) override
    {
        proton::connection_options opts;
        if (!user_.empty())
        {
            opts.sasl_allowed_mechs("PLAIN");
            opts.sasl_allow_insecure_mechs(true);
            opts.user(user_).password(pass_);
        }
        else
        {
            opts.sasl_allowed_mechs("ANONYMOUS");
        }
        proton::reconnect_options ropts;
        ropts.delay(proton::duration(2000));
        ropts.max_delay(proton::duration(30'000));
        ropts.max_attempts(0);
        opts.reconnect(ropts);
        c.connect(url_, opts);
    }

    void on_connection_open(proton::connection& conn) override
    {
        // proton re-opens both links itself on a reconnect (the dynamic
        // receiver gets a fresh address via on_receiver_open); opening them
        // again here added a sender and a reply queue per broker restart.
        if (conn.reconnected())
            return;
        // Open request sender with ANYCAST so Artemis uses a proper queue.
        proton::sender_options sopts;
        sopts.target(proton::target_options().capabilities({proton::symbol("queue")}));
        sender_ = conn.open_sender(req_queue_, sopts);

        // Dynamic receiver: Artemis assigns a unique temporary address.
        // Responses from the controller go to reply_to=<this address>.
        // Because the address is unique per-connection, no competing consumers.
        proton::receiver_options ropts;
        ropts.source(proton::source_options().dynamic(true));
        conn.open_receiver("", ropts);
    }

    void on_receiver_open(proton::receiver& r) override
    {
        reply_addr_ = r.source().address();
        spdlog::info("[TaskAmqpChannel] connected, reply_to={}", reply_addr_);
        wq_.store(&r.work_queue());
        std::vector<std::string> held;
        {
            std::lock_guard<std::mutex> lk(mu_);
            ready_ = true;
            held.swap(held_);
            ready_cv_.notify_all();
        }
        for (const auto& body : held)
            sendNow(body);
        if (!held.empty())
            spdlog::info("[TaskAmqpChannel] sent {} held message(s) after reconnect", held.size());
    }

    void on_message(proton::delivery& d, proton::message& msg) override
    {
        d.accept();
        try
        {
            std::string                 b = proton::get<std::string>(msg.body());
            auto                        j = nlohmann::json::parse(b);
            std::lock_guard<std::mutex> lk(mu_);
            if (!pending_corr_.empty()
                && (j.value("request_id", "") == pending_corr_
                    || j.value("correlation_id", "") == pending_corr_))
            {
                pending_result_ = {true, b};
                pending_corr_.clear();
                result_cv_.notify_all();
            }
        }
        catch (...)
        {
        }
    }

    void on_transport_error(proton::transport& t) override
    {
        spdlog::warn("[TaskAmqpChannel] transport error: {}", t.error().what());
        std::lock_guard<std::mutex> lk(mu_);
        ready_ = false;
        reply_addr_.clear();
    }

    void on_connection_error(proton::connection& c) override
    {
        spdlog::warn("[TaskAmqpChannel] connection error: {}", c.error().what());
    }

private:
    static constexpr size_t MAX_HELD      = 64;
    static constexpr int    STOP_GRACE_MS = 3000;

    // proton thread only.
    void sendNow(const std::string& body)
    {
        if (!sender_)
            return;
        proton::message msg;
        msg.body(body);
        msg.content_type("application/json");
        sender_.send(msg);
    }

    std::string url_, user_, pass_, req_queue_, resp_queue_;

    std::unique_ptr<proton::container> container_;
    std::thread                        thread_;

    proton::sender                   sender_;
    std::atomic<proton::work_queue*> wq_{nullptr};
    std::string                      reply_addr_;

    std::mutex               mu_;
    std::condition_variable  ready_cv_;
    std::condition_variable  result_cv_;
    bool                     ready_{false};
    bool                     stopped_{false};
    std::vector<std::string> held_;  ///< sends made while not connected (mu_)
    std::string              pending_corr_;
    AmqpResponse             pending_result_;
};

}  // namespace acq

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
