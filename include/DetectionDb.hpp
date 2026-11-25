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
#pragma once
/**
 * @file DetectionDb.hpp
 * @brief Asynchronous PostgreSQL writer for Detection records.
 *
 * DetectionDb accepts Detection objects from any thread via push() and
 * writes them to PostgreSQL in background batches, decoupling the sweep
 * loop from database I/O latency.
 *
 * Batches are flushed when either:
 * - @p batch_size detections have accumulated, or
 * - @p flush_interval elapsed since the last flush.
 */
#include "Types.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <deque>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <au/prefix.hh>
#include <au/units/seconds.hh>
#include <pqxx/pqxx>

namespace acq
{

/**
 * @brief Writes Detection records to PostgreSQL in background batches.
 *
 * Non-copyable.  push() is safe to call from any thread.
 * The destructor flushes remaining records and joins the writer thread.
 */
class DetectionDb
{
public:
    /**
     * @brief Open a PostgreSQL connection and start the writer thread.
     * @param conn_str        libpqxx connection string.
     * @param batch_size      Flush after accumulating this many records.
     * @param flush_interval  Flush after this interval even if batch is not full.
     * @throws std::exception if the initial connection fails.
     */
    explicit DetectionDb(const std::string&         conn_str,
                         int                        batch_size     = 100,
                         au::QuantityD<au::Seconds> flush_interval = au::milli(au::seconds)(500.0));
    ~DetectionDb();

    DetectionDb(const DetectionDb&)            = delete;
    DetectionDb& operator=(const DetectionDb&) = delete;

    /**
     * @brief Enqueue a Detection for writing.
     *
     * Returns immediately; the detection is written asynchronously.
     * Thread-safe.
     *
     * @param d Detection to persist.
     */
    void push(const Detection& d);

    /// True while the writer holds a working connection.
    bool connected() const;
    /// Detections dropped because the queue was full during an outage.
    uint64_t dropped() const;

private:
    std::string                conn_str_;
    std::unique_ptr<pqxx::connection> conn_;  ///< writer thread only; null = disconnected
    std::deque<Detection>      queue_;
    std::mutex                 mutex_;
    std::condition_variable    cv_;
    std::atomic<bool>          stopped_{false};
    std::thread                thread_;
    int                        batch_size_;
    au::QuantityD<au::Seconds> flush_interval_;

    void workerLoop();
    bool tryConnect();     ///< writer thread; honours the reconnect backoff
    void ensureSchema();

    static constexpr size_t               kMaxQueued = 20'000;
    std::atomic<bool>                     connected_{false};
    std::atomic<uint64_t>                 dropped_{0};
    bool                                  warned_ = false;  // writer thread
    int                                   retry_s_ = 1;     // writer thread
    std::chrono::steady_clock::time_point next_connect_{};  // writer thread
    void flush(std::vector<Detection>& batch);
};

}  // namespace acq

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
