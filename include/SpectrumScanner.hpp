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
 * @file SpectrumScanner.hpp
 * @brief Frequency sweep loop with persistence filter and noise floor EMA.
 *
 * SpectrumScanner drives an IqSource in a background thread, running
 * FftProcessor on each dwell and applying a persistence filter before
 * invoking the DetectionCallback.
 *
 * ## Persistence filter
 * A candidate detection must appear within ±PERSIST_FREQ_QUANT_HZ of the
 * same persistence entry in at least PERSIST_MIN_HITS sweeps (≤
 * PERSIST_MAX_MISSES misses in between).
 * Signals with SNR ≥ PERSIST_BYPASS_SNR_DB or PAPR ≥ PERSIST_BYPASS_PAPR_DB (strong, obvious
 * signals such as FM broadcast or LTE) bypass persistence and emit on the first sweep.
 *
 * ## Noise floor
 * Each dwell's floor comes from a NoiseFloorEstimator (<noise_floor>): per
 * block ordered statistics plus minimum statistics over the last sweeps of
 * the same entry, and a detection margin for a fixed false-alarm probability.
 * method "cfar" keeps the censored CA-CFAR with a fixed threshold_db.
 */
#include "FftProcessor.hpp"
#include "IqSource.hpp"
#include "NoiseFloorEstimator.hpp"
#include "SweepConfig.hpp"
#include "Types.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

#include <au/units/hertz.hh>

namespace acq
{

/**
 * @brief Drives an IqSource through a frequency sweep and fires detections.
 *
 * Non-copyable.  Call start() to launch the sweep thread and stop() to
 * join it.  The DetectionCallback is invoked from the sweep thread; keep
 * it fast or hand off to a queue.
 */
class SpectrumScanner
{
public:
    /// @brief Callback fired for each confirmed detection.
    using DetectionCallback = std::function<void(const Detection&)>;

    /**
     * @brief Construct the scanner.
     * @param cfg     Sweep configuration (frequency range, FFT params, etc.).
     * @param source  IQ dwell source (must outlive this scanner).
     * @param cb      Detection callback (invoked from the sweep thread).
     */
    SpectrumScanner(SweepConfig cfg, IqSource* source, DetectionCallback cb);
    ~SpectrumScanner();

    SpectrumScanner(const SpectrumScanner&)            = delete;
    SpectrumScanner& operator=(const SpectrumScanner&) = delete;

    /// @brief Open the IqSource and launch the sweep thread.
    void start();

    /// @brief Signal stop, close the IqSource, and join the sweep thread.
    void stop();

    /// Receives RF_NOISE_MAP data when cfg.noise_map.enabled (call before
    /// start()). Invoked from the processing thread of the dwell's channel.
    void setNoiseMapCallback(std::function<void(const NoiseMap&)> cb)
    {
        map_cb_ = std::move(cb);
    }

private:
    SweepConfig       cfg_;
    IqSource*         source_;
    DetectionCallback cb_;

    std::vector<std::unique_ptr<FftProcessor>> processors_;
    std::atomic<bool>                          running_{false};
    std::thread                                thread_;

    /// Per-channel noise-floor estimator (keeps its own sweep history per entry).
    std::vector<std::unique_ptr<NoiseFloorEstimator>> noise_;

    /// Noise-map accumulation per channel and scan entry (cfg.noise_map).
    struct NoiseAccum
    {
        int                sweeps = 0, lo = 0, hi = -1, fft = 0;
        std::vector<float> floor_sum, quiet, occ;
    };

    std::vector<std::unordered_map<uint64_t, NoiseAccum>> ch_map_;
    std::function<void(const NoiseMap&)>                  map_cb_;

    void accumulateNoiseMap(int                     ch,
                            uint64_t                key,
                            double                  center_hz,
                            double                  sample_rate_hz,
                            const FftProcessor&     proc,
                            const NoiseFloorResult& nf);

    /// Persistence filter state: track hit/miss counts per (channel, centre_hz, freq).
    struct PersistEntry
    {
        int   hits{0};          ///< Consecutive sweeps the signal was detected.
        int   misses{0};        ///< Consecutive sweeps the signal was absent.
        float peak_db{-300.f};  ///< Highest observed peak power (dBFS).
    };

    std::vector<std::unordered_map<uint64_t, std::unordered_map<uint64_t, PersistEntry>>>
        ch_persist_;

    /// A detection matches the nearest persistence entry within this distance.
    /// Wide enough to tolerate sub-bin interpolation jitter across sweeps.
    static constexpr uint64_t PERSIST_FREQ_QUANT_HZ = 10'000;
    /// Minimum confirmed hits before a detection callback fires.
    static constexpr int PERSIST_MIN_HITS = 2;
    /// Max consecutive misses before a persistence entry is evicted.
    static constexpr int PERSIST_MAX_MISSES = 4;
    /// Signals with PAPR ≥ this threshold bypass persistence (emit on first sweep).
    static constexpr float PERSIST_BYPASS_PAPR_DB = 15.0f;
    /// … or SNR ≥ this (peak above the estimated floor).
    static constexpr float PERSIST_BYPASS_SNR_DB = 25.0f;

    void sweepLoop();

    /**
     * @brief Process one dwell and return confirmed detections.
     * @param ch         Channel index.
     * @param center_hz  LO centre frequency for this dwell.
     * @param samples    IQ samples for this dwell.
     * @return Detections that passed the persistence filter.
     */
    std::vector<Detection> processDwell(int                                     ch,
                                        au::QuantityD<au::Hertz>                center_hz,
                                        au::QuantityD<au::Hertz>                sample_rate,
                                        const std::vector<std::complex<float>>& samples);
};

}  // namespace acq

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
