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
#include "AmqpPublisher.hpp"

#include <nlohmann/json.hpp>
#include <proton/reconnect_options.hpp>
#include <sdr/Base64.hpp>
#include <spdlog/spdlog.h>

#include <chrono>

#include <au/units/hertz.hh>

namespace acq
{

using json = nlohmann::json;
using namespace std::chrono;

AmqpPublisher::AmqpPublisher(std::string url,
                             std::string username,
                             std::string password,
                             std::string topic,
                             std::string scanner_id)
    : url_(std::move(url))
    , username_(std::move(username))
    , password_(std::move(password))
    , topic_(std::move(topic))
    , scanner_id_(std::move(scanner_id))
{
}

AmqpPublisher::~AmqpPublisher()
{
    stop();
}

void AmqpPublisher::start()
{
    container_ = new proton::container(*this);
    thread_    = std::thread(
        [this]
        {
            container_->run();
        });
}

void AmqpPublisher::stop()
{
    stopping_ = true;
    if (container_)
    {
        if (auto* wq = work_queue_.load())
            wq->add(
                [this]
                {
                    flush();  // whatever the link has credit for goes out before the close
                    sender_.connection().close();
                });
        else
            // Connection never reached on_sender_open, so there's no work
            // queue to post a close through. reconnect_options above sets
            // max_attempts(0) — infinite retries — so without this,
            // container_->run() never returns and thread_.join() blocks
            // forever.
            container_->stop();
        if (thread_.joinable())
            thread_.join();
        work_queue_.store(nullptr);
        delete container_;
        container_ = nullptr;
    }
}

void AmqpPublisher::enqueue(proton::message msg)
{
    if (stopping_)
        return;
    {
        std::lock_guard lk(outbox_mu_);
        // Held until the link is up and has credit (ACQ-16): startup before
        // the broker answers, a reconnect, or a consumer that stopped taking
        // messages. Bounded, newest kept.
        if (outbox_.size() >= kDefaultMaxQueued)
        {
            outbox_.pop_front();
            const auto n = dropped_.fetch_add(1) + 1;
            if (n == 1 || n % 1000 == 0)
                spdlog::warn(
                    "[AmqpPublisher] {}: broker not taking messages, {} dropped (oldest first)",
                    topic_,
                    n);
        }
        outbox_.push_back(std::move(msg));
    }
    if (auto* wq = work_queue_.load())
        wq->add(
            [this]
            {
                flush();
            });
}

void AmqpPublisher::flush()
{
    if (!sender_ || !sender_.active())
        return;
    while (sender_.credit() > 0)
    {
        proton::message msg;
        {
            std::lock_guard lk(outbox_mu_);
            if (outbox_.empty())
                return;
            msg = std::move(outbox_.front());
            outbox_.pop_front();
        }
        sender_.send(msg);
    }
}

void AmqpPublisher::publish(const Detection& d)
{
    enqueue(makeMessage(d));
}

std::string AmqpPublisher::noiseMapJson(const NoiseMap& m)
{
    const auto ms   = duration_cast<milliseconds>(m.timestamp.time_since_epoch()).count();
    json       body = {{"msg_type", "RF_NOISE_MAP"},
                       {"schema_version", "1.0"},
                       {"timestamp_ms", ms},
                       {"scanner_id", m.scanner_id},
                       {"channel", m.channel},
                       {"center_freq_hz", m.center_freq_hz},
                       {"sample_rate_hz", m.sample_rate_hz},
                       {"f_start_hz", m.f_start_hz},
                       {"point_hz", m.point_hz},
                       {"sweeps", m.sweeps},
                       {"floor_db", m.floor_db},
                       {"quiet_db", m.quiet_db},
                       {"occupancy", m.occupancy}};
    if (m.dbm_offset_db)
        body["dbm_offset_db"] = *m.dbm_offset_db;
    return body.dump();
}

void AmqpPublisher::publish(const NoiseMap& m)
{
    proton::message msg;
    msg.body(noiseMapJson(m));
    msg.content_type("application/json");
    enqueue(std::move(msg));
}

void AmqpPublisher::on_container_start(proton::container& c)
{
    proton::connection_options opts;
    if (!username_.empty())
    {
        opts.sasl_allowed_mechs("PLAIN");
        opts.sasl_allow_insecure_mechs(true);
        opts.user(username_).password(password_);
    }
    else
    {
        opts.sasl_allowed_mechs("ANONYMOUS");
    }
    // Without this, a failed initial connection (e.g. broker not up yet) is
    // permanent — proton tears down the container and the publisher never
    // recovers. Retry indefinitely with backoff, matching TaskAmqpChannel.
    proton::reconnect_options ropts;
    ropts.delay(proton::duration(2000));
    ropts.max_delay(proton::duration(30'000));
    ropts.max_attempts(0);
    opts.reconnect(ropts);
    c.connect(url_, opts);
}

void AmqpPublisher::on_connection_open(proton::connection& conn)
{
    // On a reconnect proton re-opens the existing sender itself; opening
    // another here added one more link per broker restart.
    if (!conn.reconnected())
        conn.open_sender(topic_);
}

void AmqpPublisher::on_sender_open(proton::sender& s)
{
    sender_ = s;
    work_queue_.store(&s.work_queue());
    spdlog::info("[AmqpPublisher] connected → {}", topic_);
    flush();
}

void AmqpPublisher::on_sendable(proton::sender&)
{
    flush();
}

void AmqpPublisher::on_transport_error(proton::transport& t)
{
    spdlog::warn("[AmqpPublisher] transport error: {}", t.error().what());
}

void AmqpPublisher::on_connection_error(proton::connection& c)
{
    spdlog::warn("[AmqpPublisher] connection error: {}", c.error().what());
}

proton::message AmqpPublisher::makeMessage(const Detection& d) const
{
    auto ms = duration_cast<milliseconds>(d.timestamp.time_since_epoch()).count();

    json body = {{"msg_type", "RF_DETECTION"},
                 {"schema_version", SCHEMA_VERSION},
                 {"timestamp_ms", ms},
                 {"scanner_id", d.scanner_id},
                 {"center_freq_hz", d.center_freq_hz.in(au::hertz)},
                 {"bandwidth_hz", d.bandwidth_hz.in(au::hertz)},
                 {"power_db", d.power_db},
                 {"snr_db", d.snr_db},
                 {"papr_db", d.papr_db},
                 {"noise_floor_db", d.noise_floor_db},
                 {"channel", d.channel}};
    if (d.noise_floor_dbm)
        body["noise_floor_dbm"] = *d.noise_floor_dbm;
    if (d.power_dbm)
        body["power_dbm"] = *d.power_dbm;

    // IQ snapshot: base64-encoded raw float32 bytes (~11 KB) instead of a
    // JSON float array (~28 KB). Receivers decode with sdr::base64::decodeFloats().
    // Backward-compatible decoders fall back to "iq_snapshot" (JSON array).
    if (d.iq_snapshot && !d.iq_snapshot->empty())
    {
        body["iq_snapshot_b64"] =
            sdr::base64::encode(d.iq_snapshot->data(), d.iq_snapshot->size() * sizeof(float));
        body["snapshot_sample_rate_sps"] = d.snapshot_sample_rate_sps.in(au::hertz);
    }

    proton::message msg;
    msg.body(body.dump());
    msg.content_type("application/json");
    return msg;
}

}  // namespace acq

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
