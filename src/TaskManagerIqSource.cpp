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
#include "TaskManagerIqSource.hpp"

#include "LocalAddress.hpp"
#include "TaskAmqpChannel.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
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
#include <sdr/Types.hpp>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <sstream>
#include <stdexcept>

#include <au/units/hertz.hh>

namespace acq
{

using json = nlohmann::json;
using namespace std::chrono;

using IqPacketHeader                        = sdr::IqPacketHeader;
static constexpr uint32_t IQ_MAGIC          = sdr::IQ_PACKET_MAGIC;
static constexpr uint8_t  FLAG_DWELL_CHANGE = sdr::IQ_FLAG_DWELL_CHANGE;

// ── Simple request-ID generator ───────────────────────────────────────────────
static std::string makeReqId()
{
    auto ns = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
    std::ostringstream ss;
    ss << "acq-" << std::hex << ns;
    return ss.str();
}

// ── TaskManagerIqSource ───────────────────────────────────────────────────────

TaskManagerIqSource::TaskManagerIqSource(const SweepConfig& cfg,
                                         std::string        preferred_device,
                                         bool               device_required)
    : cfg_(cfg)
    , preferred_device_(std::move(preferred_device))
    , device_required_(device_required)
    , assembler_(cfg.device.rx_channels,
                 effectiveDwellSamples(cfg.sweep, cfg.device.sample_rate.in(au::hertz)),
                 cfg.sweep.settle_samples)
    , resume_hz_(cfg.sweep.start_hz)
{
    pkt_buf_.resize(65'536);
    resetAccum();

    // Start persistent AMQP channel immediately — Artemis settles the connection
    // in ~30 s.  By the time open() is called, it will already be ready.
    amqp_ch_ = std::make_unique<TaskAmqpChannel>(cfg_.amqp.url,
                                                 cfg_.amqp.username,
                                                 cfg_.amqp.password,
                                                 cfg_.amqp.task_request_queue,
                                                 cfg_.amqp.task_response_queue,
                                                 cfg_.amqp.status_topic);
    amqp_ch_->start(60'000);
}

TaskManagerIqSource::~TaskManagerIqSource()
{
    close();
}

void TaskManagerIqSource::resetAccum()
{
    const auto& st = assembler_.stats();
    if (st.gaps || st.overflows || st.late_packets || st.malformed)
        spdlog::info("[TaskMgrSrc] stream stats: {} pkts, {} gaps ({} pkts lost), {} overflows, "
                     "{} late, {} malformed, {} samples discarded to avoid splicing",
                     st.packets,
                     st.gaps,
                     st.lost_packets,
                     st.overflows,
                     st.late_packets,
                     st.malformed,
                     st.discarded_samples);
    assembler_.reset();
}

void TaskManagerIqSource::bindUdp(uint16_t port)
{
    udp_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd_ < 0)
        throw std::runtime_error(std::string("UDP socket() failed: ") + strerror(errno));

    // Set receive timeout so next() can check the stop flag
    struct timeval tv = {.tv_sec = 0, .tv_usec = 100'000};  // 100 ms
    setsockopt(udp_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Enlarge kernel receive buffer to absorb the full sweep while waiting for
    // TASK_ACCEPTED.  At 20 MSPS (160 MB/s) a 920 MHz sweep takes ~373ms = ~59 MB.
    // Requesting 50 MB → kernel allocates min(2×50 MB, rmem_max) ≈ 100 MB effective,
    // which is enough to buffer the complete sweep before we start draining.
    int rcvbuf = 50 * 1024 * 1024;
    if (setsockopt(udp_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0)
        spdlog::warn("[TaskMgrSrc] SO_RCVBUF failed: {}", strerror(errno));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    inet_aton(cfg_.receiver.local_ip.c_str(), &addr.sin_addr);

    if (::bind(udp_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        ::close(udp_fd_);
        udp_fd_ = -1;
        throw std::runtime_error(std::string("UDP bind() failed on port ") + std::to_string(port)
                                 + ": " + strerror(errno));
    }

    spdlog::info("[TaskMgrSrc] UDP socket bound on {}:{}", cfg_.receiver.local_ip, port);
}

std::string TaskManagerIqSource::buildScanRequest(const std::string& req_id) const
{
    const double sr_hz    = cfg_.device.sample_rate.in(au::hertz);
    const double bw_hz    = cfg_.device.bandwidth_hz.in(au::hertz);
    const double start_hz = cfg_.sweep.start_hz.in(au::hertz);
    const double stop_hz  = cfg_.sweep.stop_hz.in(au::hertz);
    const double resume   = resume_hz_.in(au::hertz);

    const double step_hz  = sr_hz * cfg_.sweep.usable_bw_fraction;
    const int    dwell_ms = scanDwellMs(effectiveDwellSamples(cfg_.sweep, sr_hz), sr_hz);

    // Entries on a fixed grid, starting at the entry that contains resume_hz
    // and wrapping, so a re-submission after preemption continues the sweep
    // and every frequency is covered exactly once per task (ACQ-06). A span
    // narrower than one step (P25 voice channel) gets one centred entry.
    const auto centers = scanEntryCenters(start_hz, stop_hz, step_hz, resume);
    json       entries = json::array();
    int        step_i  = 0;
    for (double c : centers)
        entries.push_back(json{{"step", step_i++},
                               {"center_freq_hz", c},
                               {"bandwidth_hz", bw_hz},
                               {"sample_rate_sps", sr_hz},
                               {"dwell_ms", dwell_ms}});
    last_entry_hz_ = centers.back();

    std::vector<double> gains(cfg_.device.rx_channels, cfg_.device.rx_gain_db);

    auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

    // Build the streaming object; include the pre-bound port so the controller
    // can stream to it immediately without waiting for a handshake.
    // A wildcard bind address is not a destination. It used to become
    // 127.0.0.1, which only works when SdrResourceManager runs on this host;
    // use the local address of the route towards the broker instead (ACQ-07).
    std::string dest_ip = (cfg_.receiver.local_ip == "0.0.0.0") ? localAddressTowards(cfg_.amqp.url)
                                                                : cfg_.receiver.local_ip;
    json        streaming_obj = {{"dest_ip", dest_ip}};
    if (prebound_port_ > 0)
        streaming_obj["dest_ports"] = json::array({static_cast<int>(prebound_port_)});

    json req = {{"msg_type", "TASK_REQUEST_SCAN"},
                {"schema_version", "2.0"},
                {"timestamp_ms", now_ms},
                {"request_id", req_id},
                {"task_type", "SCAN"},
                {"rank", cfg_.rank},
                {"schedule", {{"mode", "CONTINUOUS"}}},
                {"rf",
                 {{"center_freq_hz", (start_hz + stop_hz) / 2.0},
                  {"bandwidth_hz", bw_hz},
                  {"sample_rate_sps", sr_hz},
                  {"rx_count", cfg_.device.rx_channels},
                  {"rx_gain_db", gains},
                  {"rx_agc", std::vector<bool>(cfg_.device.rx_channels, cfg_.device.rx_agc)},
                  {device_required_ ? "required_device" : "preferred_device", preferred_device_}}},
                {"streaming", streaming_obj},
                {"scan_params", {{"repeat", false}, {"entries", entries}}}};
    return req.dump();
}

// Returns the UDP port the controller allocated for our stream.
uint16_t TaskManagerIqSource::submitTask()
{
    std::string req_id = makeReqId();
    std::string body   = buildScanRequest(req_id);

    spdlog::info("[TaskMgrSrc] submitting SCAN task (req_id={})", req_id);

    auto resp = amqp_ch_->exchange(body, req_id, 30'000);
    if (!resp.received)
        throw std::runtime_error("Task request timed out — is the controller running?");

    auto j = json::parse(resp.body);
    // Controller sends {"status": "ACCEPTED" | "REJECTED"} not {"accepted": bool}
    bool accepted = (j.value("status", "REJECTED") == "ACCEPTED") || j.value("accepted", false);
    if (!accepted)
    {
        throw std::runtime_error("Task rejected: " + j.value("reject_reason", "unknown"));
    }
    task_id_ = j.value("task_id", "");

    // Controller allocates ports from its own pool and returns them in streams[].udp_port
    uint16_t port = 0;
    if (j.contains("streams") && !j["streams"].empty())
        port = j["streams"][0].value("udp_port", 0);
    if (port == 0)
        throw std::runtime_error("ACCEPTED response missing streams[0].udp_port");

    spdlog::info("[TaskMgrSrc] task accepted (task_id={} udp_port={})", task_id_, port);
    return port;
}

void TaskManagerIqSource::sendTaskStop()
{
    if (task_id_.empty())
        return;
    std::string req_id = makeReqId();
    auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

    json msg = {{"msg_type", "TASK_STOP"},
                {"schema_version", "2.0"},
                {"timestamp_ms", now_ms},
                {"request_id", req_id},
                {"task_id", task_id_},
                {"reason", "scanner stopping"}};
    amqp_ch_->send(msg.dump());
    spdlog::info("[TaskMgrSrc] TASK_STOP sent for task_id={}", task_id_);
    task_id_.clear();
}

int TaskManagerIqSource::queryDeviceCount()
{
    auto req_id = makeReqId();
    auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    json req    = {{"msg_type", "HEALTH_QUERY"},
                   {"schema_version", "2.0"},
                   {"timestamp_ms", now_ms},
                   {"request_id", req_id}};
    // exchange()'s readiness wait consumes its entire timeout_ms budget
    // before ever attempting to send+await a response (it returns
    // immediately if the channel isn't connected yet) -- a single 120s
    // shot isn't always enough: a cold Artemis pod has been observed
    // taking up to ~4 min to start accepting AMQP connections on a
    // Raspberry Pi. Retry in a loop with periodic warnings instead of
    // giving up after one shot, mirroring the patient-retry pattern used
    // for AMQP session bring-up elsewhere (see sdr_recon.py). Each retry
    // re-sends the same idempotent query under the same request_id, which
    // is harmless if an earlier attempt's response arrives late.
    constexpr int kStepMs    = 60'000;
    constexpr int kMaxWaitMs = 300'000;
    AmqpResponse  resp;
    int           waited = 0;
    while (waited < kMaxWaitMs)
    {
        resp = amqp_ch_->exchange(req.dump(), req_id, kStepMs);
        if (resp.received)
            break;
        waited += kStepMs;
        spdlog::warn("[TaskMgrSrc] HEALTH_QUERY not answered after {}s — "
                     "controller/broker still starting?",
                     waited / 1000);
    }
    if (!resp.received)
    {
        spdlog::warn("[TaskMgrSrc] HEALTH_QUERY timed out after {}s — assuming 1 device",
                     kMaxWaitMs / 1000);
        return 1;
    }
    try
    {
        auto j = json::parse(resp.body);
        // Controller emits "online_devices" (see MessageCodec::encodeControllerHealth
        // in SdrTaskApi) -- not "num_devices_online". A mismatched key here silently
        // falls back to the default via .value() with no warning or exception, which
        // is why band-splitting always saw "1 device" even with N genuinely online.
        int n = j["controller"].value("online_devices", 1);
        spdlog::info("[TaskMgrSrc] SdrRM reports {} online device(s)", n);
        return std::max(1, n);
    }
    catch (...)
    {
        spdlog::warn("[TaskMgrSrc] HEALTH_QUERY parse failed — assuming 1 device");
        return 1;
    }
}

void TaskManagerIqSource::open()
{
    // Pre-bind before submitting the task so the controller streams to a
    // ready socket from the first packet — eliminates the race where
    // AMQP delivery of TASK_ACCEPTED arrives after streaming has finished.
    //
    // With port=0, the OS assigns an ephemeral port (e.g. 54321); we include
    // it in the task request's dest_ports[] so the controller honours it.
    // With a fixed port in config, we bind that port and include it too.
    uint16_t local_port = static_cast<uint16_t>(cfg_.receiver.port);
    bindUdp(local_port);

    // Learn the actual bound port (important when local_port==0).
    if (local_port == 0)
    {
        struct sockaddr_in sa{};
        socklen_t          sl = sizeof(sa);
        getsockname(udp_fd_, reinterpret_cast<sockaddr*>(&sa), &sl);
        local_port = ntohs(sa.sin_port);
    }
    prebound_port_ = local_port;  // buildScanRequest() reads this

    uint16_t ctrl_port = submitTask();
    prebound_port_     = 0;

    if (ctrl_port != 0 && ctrl_port != local_port)
    {
        // Controller overrode our port (didn't support dest_ports[]) —
        // re-bind to its assignment.  Data may be lost for this sweep.
        spdlog::warn("[TaskMgrSrc] controller overrode port {} → {}; re-binding "
                     "(upgrade controller to honour dest_ports[])",
                     local_port,
                     ctrl_port);
        ::close(udp_fd_);
        udp_fd_ = -1;
        bindUdp(ctrl_port);
    }
    resetAccum();
    first_task_packet_ = true;
    sweep_done_        = false;
    running_           = true;
}

void TaskManagerIqSource::stop()
{
    running_ = false;
}

void TaskManagerIqSource::close()
{
    running_ = false;
    sendTaskStop();
    if (udp_fd_ >= 0)
    {
        ::close(udp_fd_);
        udp_fd_ = -1;
    }
}

bool TaskManagerIqSource::next(Dwell& d)
{
    // Two-phase no-data timeout:
    //  • Before the first packet of the TASK arrives the SCAN task may sit
    //    in PENDING while AnalysisApp finishes a WIDEBAND task — allow up
    //    to 30 s. (first_task_packet_ is a member set once per open(), not
    //    once per call -- see its declaration for why a local here was a
    //    bug: it silently re-granted this 30s budget after every single
    //    completed dwell, including the final call after the controller's
    //    sweep had already finished, so every successful task still ended
    //    in a guaranteed ~30s of dead waiting before being recognized as
    //    over.)
    //  • After data starts flowing the inter-dwell gap (PLL calibration) is
    //    3-4 s; allow 5 s before declaring the stream ended.
    static constexpr int INITIAL_TIMEOUT_MS = 30'000;
    static constexpr int STREAM_TIMEOUT_MS  = 5000;
    int                  no_data_ms         = 0;
    if (sweep_done_)
        return false;

    while (running_)
    {
        ssize_t n = ::recvfrom(udp_fd_, pkt_buf_.data(), pkt_buf_.size(), 0, nullptr, nullptr);
        if (n < 0)
        {
            // EINTR is not a real failure -- it just means a signal hit this
            // thread mid-syscall (observed live: proton's AMQP container
            // thread/timers deliver these periodically). Previously this fell
            // through to the generic error branch below and returned false,
            // aborting an otherwise perfectly healthy stream and forcing a
            // full task resubmit. Confirmed via strace: a session that was
            // successfully receiving ~9000 pkts/sec for 57s got killed by a
            // single EINTR and had to restart from scratch -- with the SCAN
            // task's 30s-to-first-packet budget, repeated mid-stream EINTRs
            // are enough to make a device that's genuinely streaming data
            // look like it never produces any.
            if (errno == EINTR)
                continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                no_data_ms += 100;
                // The RM reported the task over (preempted by a higher rank,
                // cancelled, failed) and the socket is drained: resubmit now.
                // Waiting out STREAM_TIMEOUT_MS cost 5 s per preemption.
                if (!task_id_.empty() && amqp_ch_->isTerminal(task_id_))
                {
                    spdlog::info("[TaskMgrSrc] task {} ended by the RM — re-submitting", task_id_);
                    return false;
                }
                int limit = first_task_packet_ ? INITIAL_TIMEOUT_MS : STREAM_TIMEOUT_MS;
                if (no_data_ms >= limit)
                {
                    spdlog::info("[TaskMgrSrc] no IQ data for {}ms — task complete, re-submitting",
                                 no_data_ms);
                    return false;
                }
                continue;
            }
            spdlog::error("[TaskMgrSrc] recvfrom error: {}", strerror(errno));
            return false;
        }
        first_task_packet_ = false;
        no_data_ms         = 0;
        // Header validation, dwell boundaries, sequence-gap / overflow
        // handling and per-channel accumulation (DwellAssembler.hpp).
        if (assembler_.push(pkt_buf_.data(), static_cast<size_t>(n), d))
        {
            // Resume after the last *completed* dwell, on the entry grid.
            const double step_hz =
                cfg_.device.sample_rate.in(au::hertz)*cfg_.sweep.usable_bw_fraction;
            const double c = d.center_hz.in(au::hertz);
            resume_hz_     = au::hertz(resumeAfterDwell(
                cfg_.sweep.start_hz.in(au::hertz), cfg_.sweep.stop_hz.in(au::hertz), step_hz, c));
            // The task's last entry is in: the sweep is over. Return false on
            // the next call instead of idling STREAM_TIMEOUT_MS (5 s) on a
            // silent socket — that dead time used to follow every sweep.
            // Header CFs are the entry centres to the Hz; the last two entries
            // can be closer than a step (the last window ends at stop).
            if (std::abs(c - last_entry_hz_) < 1e3)
                sweep_done_ = true;
            return true;
        }
    }
    return false;
}

void TaskManagerIqSource::setSweepBand(au::QuantityD<au::Hertz> start_hz,
                                       au::QuantityD<au::Hertz> stop_hz)
{
    cfg_.sweep.start_hz = start_hz;
    cfg_.sweep.stop_hz  = stop_hz;
    resume_hz_          = start_hz;
}

}  // namespace acq

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
