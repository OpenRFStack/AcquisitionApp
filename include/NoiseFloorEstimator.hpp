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
 * @file NoiseFloorEstimator.hpp
 * @brief Noise-floor estimation from a Welch PSD (DSP; an ML method can plug in).
 *
 * Both estimators take the averaged spectrum FftProcessor produced for one
 * dwell and return the noise floor per bin (dBFS) plus the detection margin
 * for a target false-alarm probability. Detection then compares every bin
 * with floor + margin.
 *
 * ## DSP (DspNoiseFloor)
 *  1. Per dwell, ordered statistic per block: the q-th percentile (default
 *     25 %) of each block of bins, bias-corrected to the noise mean. Signals
 *     occupying up to (1 − q) of a block do not move it (OS-CFAR principle).
 *  2. Minimum statistics across sweeps: per scan entry and block, the minimum
 *     of the last `history` per-dwell estimates, bias-corrected. A duty-cycled
 *     or wideband burst that covers a whole block in some sweeps is ignored
 *     as long as one sweep in the window saw that block quiet.
 *  3. Blocks are clamped to ±max_tilt_db of the band-wide percentile and
 *     interpolated linearly to a per-bin floor (coloured noise / ripple).
 *  The bias factors are exact for the averaged-periodogram model (bins ~
 *  Gamma(M, 1/M) about the noise mean, M = Welch frames) and are computed
 *  once per configuration by Monte-Carlo.
 *
 * ## ML
 *  Planned: an ONNX model predicting the floor curve from the log spectrum,
 *  behind the same interface. method "ml" currently falls back to DSP.
 *
 * ## Threshold
 *  margin_db = 10·log10(Q(1 − pfa)), Q the quantile of Gamma(M, 1/M): the
 *  per-bin false-alarm probability stays at `pfa` for every dwell length
 *  (a fixed 10 dB was too loose for one frame and far too strict for 31).
 */
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace acq
{

/// One dwell's spectrum, as FftProcessor holds it after computeSpectrum().
struct NoiseFloorInput
{
    const float* power_db  = nullptr;  ///< fft_size bins, dBFS, fftshift layout
    int          fft_size  = 0;
    int          start_bin = 0;  ///< examined range, inclusive
    int          end_bin   = 0;
    int          n_frames  = 1;  ///< Welch frames averaged (M)
    uint64_t     key       = 0;  ///< scan-entry key (centre Hz) for the time history
};

struct NoiseFloorResult
{
    std::vector<float> floor_db;           ///< per bin (fft_size), valid in [start_bin, end_bin]
    float              band_floor_db = 0;  ///< median of floor_db over the range
    float              margin_db     = 0;  ///< detection margin above the floor (dB)
    bool               ok            = false;
};

class NoiseFloorEstimator
{
public:
    virtual ~NoiseFloorEstimator()                               = default;
    virtual NoiseFloorResult estimate(const NoiseFloorInput& in) = 0;
    virtual const char*      name() const                        = 0;
};

/// Detection margin (dB) for per-bin false-alarm probability @p pfa with
/// @p n_frames averaged Welch frames.
float noiseMarginDb(int n_frames, double pfa);

struct DspNoiseFloorConfig
{
    double percentile  = 0.25;  ///< per-block ordered statistic (0, 1)
    int    block_bins  = 64;    ///< bins per block
    int    history     = 8;     ///< sweeps in the minimum-statistics window (1 = off)
    float  max_tilt_db = 6.f;   ///< block floor clamp around the band percentile
    double pfa         = 3e-3;  ///< per-bin false-alarm probability
};

class DspNoiseFloor : public NoiseFloorEstimator
{
public:
    explicit DspNoiseFloor(DspNoiseFloorConfig cfg = {});
    NoiseFloorResult estimate(const NoiseFloorInput& in) override;

    const char* name() const override
    {
        return "dsp";
    }

    /// Mean of (estimate − true floor) in dB for the noise-only model, after
    /// correction — exposed for tests (should be ≈ 0).
    struct Bias
    {
        float              percentile_db;  ///< E[10log10(block percentile / mean)]
        float              min_db;         ///< extra E[min over d sweeps] shift, per d = 1..history
        std::vector<float> min_by_d;
    };

    const Bias& bias(int n_frames, int block_bins);

private:
    DspNoiseFloorConfig                 cfg_;
    std::map<std::pair<int, int>, Bias> bias_cache_;  ///< (M, block bins) → bias
    /// key → per block history of corrected per-dwell estimates (newest last)
    std::unordered_map<uint64_t, std::vector<std::vector<float>>> history_;
    std::vector<float>                                            scratch_;
};

/// Factory: "dsp" (default). "ml" is reserved for the planned ONNX model and
/// falls back to DSP with a warning.
std::unique_ptr<NoiseFloorEstimator> makeNoiseFloorEstimator(const std::string&         method,
                                                             const std::string&         model_path,
                                                             const DspNoiseFloorConfig& dsp);

}  // namespace acq

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
