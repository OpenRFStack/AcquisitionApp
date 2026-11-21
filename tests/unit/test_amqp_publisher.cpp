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
// AmqpPublisher against an in-process loopback AMQP peer that controls link
// credit, so a slow, stalled or restarting broker can be reproduced (ACQ-16).

#include "AmqpPublisher.hpp"
#include "GpsCache.hpp"
#include "P25GrantConsumer.hpp"
#include "TaskAmqpChannel.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <proton/connection.hpp>
#include <proton/container.hpp>
#include <proton/delivery.hpp>
#include <proton/listen_handler.hpp>
#include <proton/listener.hpp>
#include <proton/message.hpp>
#include <proton/messaging_handler.hpp>
#include <proton/receiver.hpp>
#include <proton/receiver_options.hpp>
#include <proton/sender.hpp>
#include <proton/work_queue.hpp>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <au/units/hertz.hh>

using namespace std::chrono_literals;

namespace
{

/// Loopback AMQP peer: accepts one sender link and grants only the credit the
/// test asks for.
class TestPeer
    : public proton::messaging_handler
    , public proton::listen_handler
{
public:
    explicit TestPeer(int initial_credit, int port = 0)
        : credit_(initial_credit)
        , want_port_(port)
    {
        thread_ = std::thread(
            [this]
            {
                container_.run();
            });
        std::unique_lock lk(mu_);
        cv_.wait_for(lk,
                     5s,
                     [this]
                     {
                         return port_ != 0 || failed_;
                     });
    }

    ~TestPeer()
    {
        shutdown();
    }

    void shutdown()
    {
        if (!thread_.joinable())
            return;
        container_.stop();
        thread_.join();
    }

    int port() const
    {
        return port_;
    }

    bool linked()
    {
        std::lock_guard lk(mu_);
        return wq_ != nullptr;
    }

    void grant(int n)
    {
        proton::work_queue* wq;
        {
            std::lock_guard lk(mu_);
            wq = wq_;
        }
        ASSERT_NE(wq, nullptr);
        wq->add(
            [this, n]
            {
                rcv_.add_credit(n);
            });
    }

    /// Wait until `n` messages arrived or `timeout` passed; returns the count.
    size_t waitFor(size_t n, std::chrono::milliseconds timeout)
    {
        std::unique_lock lk(mu_);
        cv_.wait_for(lk,
                     timeout,
                     [&]
                     {
                         return bodies_.size() >= n;
                     });
        return bodies_.size();
    }

    int links()
    {
        std::lock_guard lk(mu_);
        return links_;
    }

    std::vector<std::string> bodies()
    {
        std::lock_guard lk(mu_);
        return bodies_;
    }

    // listen_handler
    void on_open(proton::listener& l) override
    {
        std::lock_guard lk(mu_);
        port_ = l.port();
        cv_.notify_all();
    }

    void on_error(proton::listener&, const std::string&) override
    {
        std::lock_guard lk(mu_);
        failed_ = true;
        cv_.notify_all();
    }

    // messaging_handler
    void on_container_start(proton::container& c) override
    {
        listener_ = c.listen("127.0.0.1:" + std::to_string(want_port_), *this);
    }

    void on_receiver_open(proton::receiver& r) override
    {
        r.open(proton::receiver_options().credit_window(0));
        if (credit_ > 0)
            r.add_credit(credit_);
        std::lock_guard lk(mu_);
        ++links_;
        rcv_ = r;
        wq_  = &r.work_queue();
    }

    void on_sender_open(proton::sender& s) override
    {
        s.open();
        std::lock_guard lk(mu_);
        ++consumer_links_;
    }

    int consumerLinks()
    {
        std::lock_guard lk(mu_);
        return consumer_links_;
    }

    void on_error(const proton::error_condition&) override
    {
    }  // aborted on shutdown

    void on_message(proton::delivery&, proton::message& m) override
    {
        std::lock_guard lk(mu_);
        bodies_.push_back(proton::get<std::string>(m.body()));
        cv_.notify_all();
    }

private:
    int                      credit_;
    int                      want_port_;
    std::mutex               mu_;
    std::condition_variable  cv_;
    int                      port_{0};
    bool                     failed_{false};
    int                      links_{0};
    int                      consumer_links_{0};
    proton::receiver         rcv_;
    proton::work_queue*      wq_{nullptr};
    std::vector<std::string> bodies_;
    proton::listener         listener_;
    proton::container        container_{*this};
    std::thread              thread_;
};

acq::Detection detection(int seq, size_t snapshot_floats = 0)
{
    acq::Detection d;
    d.timestamp      = std::chrono::system_clock::now();
    d.center_freq_hz = au::hertz(100e6 + seq);
    d.bandwidth_hz   = au::hertz(12.5e3);
    d.scanner_id     = "t";
    d.channel        = seq;  // sequence number, read back from the JSON
    if (snapshot_floats)
    {
        d.iq_snapshot = std::make_shared<const std::vector<float>>(snapshot_floats, 0.1f);
        d.snapshot_sample_rate_sps = au::hertz(20e6);
    }
    return d;
}

std::string url(int port)
{
    return "amqp://127.0.0.1:" + std::to_string(port);
}

bool waitUntil(const std::function<bool()>& pred, std::chrono::milliseconds timeout)
{
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end)
    {
        if (pred())
            return true;
        std::this_thread::sleep_for(5ms);
    }
    return pred();
}

int seqOf(const std::string& body)
{
    return nlohmann::json::parse(body).at("channel").get<int>();
}

}  // namespace

// Detections published between start() and the sender link opening (startup,
// broker not up yet) used to be dropped without a trace.
TEST(AmqpPublisher, ACQ16_PublishBeforeLinkUpIsDelivered)
{
    TestPeer peer(1000);
    ASSERT_NE(peer.port(), 0);
    acq::AmqpPublisher pub(url(peer.port()), "", "", "rf.detections", "t");
    pub.start();
    for (int i = 0; i < 5; ++i)
        pub.publish(detection(i));
    const size_t got = peer.waitFor(5, 3s);
    std::printf("ACQ16 before-link: delivered %zu of 5\n", got);
    EXPECT_EQ(got, 5u);
    pub.stop();
}

// A broker that grants no credit (stalled consumer, full queue) made proton
// buffer every message: memory grew without bound with ~11 KB per detection.
TEST(AmqpPublisher, ACQ16_StalledBrokerQueueIsBounded)
{
    TestPeer peer(0);
    ASSERT_NE(peer.port(), 0);
    acq::AmqpPublisher pub(url(peer.port()), "", "", "rf.detections", "t");
    pub.start();
    ASSERT_TRUE(waitUntil(
        [&]
        {
            return peer.linked();
        },
        3s));
    std::this_thread::sleep_for(100ms);  // sender side sees the link open

    constexpr int N = 3000;
    for (int i = 0; i < N; ++i)
        pub.publish(detection(i, 2048));
    std::this_thread::sleep_for(200ms);
    peer.grant(100'000);
    peer.waitFor(acq::AmqpPublisher::kDefaultMaxQueued, 3s);
    std::this_thread::sleep_for(200ms);
    const auto bodies = peer.bodies();
    std::printf("ACQ16 stalled: %d published, %zu were held and delivered\n", N, bodies.size());
    ASSERT_FALSE(bodies.empty());
    EXPECT_LE(bodies.size(), acq::AmqpPublisher::kDefaultMaxQueued);
    // Newest kept: the last published detection is the last delivered.
    EXPECT_EQ(seqOf(bodies.back()), N - 1);
    EXPECT_EQ(pub.dropped(), N - bodies.size());
    pub.stop();
}

// Detections published while the broker restarts are held and delivered on
// reconnect instead of being sent into the dead link.
TEST(AmqpPublisher, ACQ16_BrokerRestartKeepsQueuedDetections)
{
    auto      peer = std::make_unique<TestPeer>(1000);
    const int port = peer->port();
    ASSERT_NE(port, 0);
    acq::AmqpPublisher pub(url(port), "", "", "rf.detections", "t");
    pub.start();
    ASSERT_TRUE(waitUntil(
        [&]
        {
            return peer->linked();
        },
        3s));
    pub.publish(detection(0));
    ASSERT_EQ(peer->waitFor(1, 2s), 1u);

    peer.reset();  // broker goes away
    std::this_thread::sleep_for(300ms);
    for (int i = 1; i <= 10; ++i)
        pub.publish(detection(i));

    peer = std::make_unique<TestPeer>(1000, port);  // broker back on the same port
    ASSERT_EQ(peer->port(), port);
    const size_t got = peer->waitFor(10, 8s);
    std::printf("ACQ16 restart: delivered %zu of 10 after reconnect\n", got);
    EXPECT_EQ(got, 10u);
    std::this_thread::sleep_for(200ms);
    std::printf("ACQ16 restart: %d sender link(s) after reconnect\n", peer->links());
    EXPECT_EQ(peer->links(), 1);  // the restored link only, not one more per reconnect
    pub.stop();
}

// Subscribers had the same reconnect fault: proton restores the receiver and
// on_connection_open opened a second one, so a topic delivered every message
// twice after each broker restart. `start` starts a subscriber on the URL and
// returns its stop function; the result is its link count after one restart.
int subscriberLinksAfterRestart(
    const std::function<std::function<void()>(const std::string&)>& start)
{
    auto      peer = std::make_unique<TestPeer>(0);
    const int port = peer->port();
    if (port == 0)
        return -1;
    auto stop = start(url(port));
    if (!waitUntil(
            [&]
            {
                return peer->consumerLinks() == 1;
            },
            3s))
        return -2;
    peer.reset();
    std::this_thread::sleep_for(300ms);
    peer = std::make_unique<TestPeer>(0, port);
    if (peer->port() != port)
        return -3;
    if (!waitUntil(
            [&]
            {
                return peer->consumerLinks() >= 1;
            },
            5s))
        return -4;
    std::this_thread::sleep_for(300ms);  // a duplicate opens in the same batch
    const int links = peer->consumerLinks();
    stop();
    return links;
}

TEST(AmqpReconnect, ACQ16_GpsCacheHasOneLinkAfterBrokerRestart)
{
    std::unique_ptr<acq::GpsCache> gps;
    const int                      links = subscriberLinksAfterRestart(
        [&](const std::string& u)
        {
            gps = std::make_unique<acq::GpsCache>(u, "", "");
            gps->start();
            return std::function<void()>(
                [&]
                {
                    gps->stop();
                });
        });
    std::printf("ACQ16 GpsCache: %d subscriber link(s) after reconnect\n", links);
    EXPECT_EQ(links, 1);
}

TEST(AmqpReconnect, ACQ16_P25GrantConsumerHasOneLinkAfterBrokerRestart)
{
    std::unique_ptr<acq::P25GrantConsumer> p25;
    const int                              links = subscriberLinksAfterRestart(
        [&](const std::string& u)
        {
            p25 = std::make_unique<acq::P25GrantConsumer>(u,
                                                          "",
                                                          "",
                                                          "rf.p25.grants",
                                                          [](const acq::P25Grant&)
                                                          {
                                                          });
            p25->start();
            return std::function<void()>(
                [&]
                {
                    p25->stop();
                });
        });
    std::printf("ACQ16 P25GrantConsumer: %d subscriber link(s) after reconnect\n", links);
    EXPECT_EQ(links, 1);
}

// TASK_STOP is fire-and-forget. While the channel reconnected, send() returned
// without a trace, so the RM kept the task -- and its radio -- until the task
// duration ran out. Stops are now held and sent once the links are back.
TEST(TaskAmqpChannel, TaskStopDuringReconnectIsDelivered)
{
    auto      peer = std::make_unique<TestPeer>(1000);
    const int port = peer->port();
    ASSERT_NE(port, 0);
    acq::TaskAmqpChannel ch(url(port), "", "", "sdr.tasks", "");
    ch.start(3000);
    ASSERT_TRUE(waitUntil(
        [&]
        {
            return peer->linked();
        },
        3s));
    ch.send(R"({"msg_type":"TASK_STOP","task_id":"a"})");
    ASSERT_EQ(peer->waitFor(1, 2s), 1u);

    peer.reset();  // broker restarts
    std::this_thread::sleep_for(300ms);
    ch.send(R"({"msg_type":"TASK_STOP","task_id":"b"})");

    peer = std::make_unique<TestPeer>(1000, port);
    ASSERT_EQ(peer->port(), port);
    const size_t got = peer->waitFor(1, 8s);
    std::printf("TASK_STOP during reconnect: %zu of 1 delivered\n", got);
    ASSERT_EQ(got, 1u);
    EXPECT_NE(peer->bodies()[0].find("\"b\""), std::string::npos);
}

// close() sends TASK_STOP and the channel is destroyed right after it: the
// stop must reach the wire before the connection closes.
TEST(TaskAmqpChannel, TaskStopRightBeforeShutdownIsDelivered)
{
    TestPeer peer(1000);
    ASSERT_NE(peer.port(), 0);
    {
        acq::TaskAmqpChannel ch(url(peer.port()), "", "", "sdr.tasks", "");
        ch.start(3000);
        ASSERT_TRUE(waitUntil(
            [&]
            {
                return peer.linked();
            },
            3s));
        ch.send(R"({"msg_type":"TASK_STOP","task_id":"c"})");
    }  // ~TaskAmqpChannel
    EXPECT_EQ(peer.waitFor(1, 2s), 1u);
}

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
