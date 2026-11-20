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
#include "NoiseFloorEstimator.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <random>

namespace acq
{

namespace
{
// Regularized lower incomplete gamma P(a, x) (Numerical Recipes gser/gcf).
double gammaP(double a, double x)
{
    if (x <= 0.0)
        return 0.0;
    const double gln = std::lgamma(a);
    if (x < a + 1.0)
    {
        double ap = a, sum = 1.0 / a, del = sum;
        for (int n = 0; n < 500; ++n)
        {
            ap += 1.0;
            del *= x / ap;
            sum += del;
            if (std::abs(del) < std::abs(sum) * 1e-14)
                break;
        }
        return sum * std::exp(-x + a * std::log(x) - gln);
    }
    double b = x + 1.0 - a, c = 1.0 / 1e-300, d = 1.0 / b, h = d;
    for (int i = 1; i < 500; ++i)
    {
        const double an = -i * (i - a);
        b += 2.0;
        d = an * d + b;
        if (std::abs(d) < 1e-300)
            d = 1e-300;
        c = b + an / c;
        if (std::abs(c) < 1e-300)
            c = 1e-300;
        d                = 1.0 / d;
        const double del = d * c;
        h *= del;
        if (std::abs(del - 1.0) < 1e-14)
            break;
    }
    return 1.0 - std::exp(-x + a * std::log(x) - gln) * h;
}

// Upper quantile of Gamma(M, 1/M) (mean 1): x with P(X > x) = pfa.
double gammaUpperQuantile(int m, double pfa)
{
    const double a  = std::max(1, m);
    double       lo = 1.0, hi = 2.0;
    while (1.0 - gammaP(a, hi * a) > pfa)
        hi *= 2.0;
    for (int i = 0; i < 100; ++i)
    {
        const double mid                           = 0.5 * (lo + hi);
        (1.0 - gammaP(a, mid * a) > pfa ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

float percentileOf(std::vector<float>& v, size_t n, double q)
{
    const size_t k = std::min(n - 1, (size_t)std::floor(q * (double)(n - 1) + 0.5));
    std::nth_element(v.begin(), v.begin() + (ptrdiff_t)k, v.begin() + (ptrdiff_t)n);
    return v[k];
}
}  // namespace

float noiseMarginDb(int n_frames, double pfa)
{
    pfa = std::clamp(pfa, 1e-12, 0.5);
    return (float)(10.0 * std::log10(gammaUpperQuantile(std::max(1, n_frames), pfa)));
}

DspNoiseFloor::DspNoiseFloor(DspNoiseFloorConfig cfg)
    : cfg_(cfg)
{
    cfg_.percentile = std::clamp(cfg_.percentile, 0.01, 0.99);
    cfg_.block_bins = std::max(8, cfg_.block_bins);
    cfg_.history    = std::clamp(cfg_.history, 1, 64);
}

// Bias of the estimators under the noise-only model, by Monte-Carlo: bins
// ~ Gamma(M, 1/M) (mean 1). Done once per (M, block size).
const DspNoiseFloor::Bias& DspNoiseFloor::bias(int n_frames, int block_bins)
{
    const auto key = std::make_pair(n_frames, block_bins);
    if (auto it = bias_cache_.find(key); it != bias_cache_.end())
        return it->second;
    std::mt19937                    rng(12'345u + (unsigned)n_frames * 977u + (unsigned)block_bins);
    std::gamma_distribution<double> g(std::max(1, n_frames), 1.0 / std::max(1, n_frames));
    // Fixed work per table (~100 k draws, bias known to ~0.02 dB): the spread of a percentile over
    // n bins shrinks as 1/√n, so large blocks need fewer trials for the same
    // precision — the band-wide table (thousands of bins) was 10 M draws on
    // the sweep thread.
    const int trials = std::clamp(200 * 64 / std::max(1, block_bins), 24, 200);
    std::vector<float>              blk((size_t)block_bins);
    std::vector<std::vector<float>> est((size_t)trials, std::vector<float>((size_t)cfg_.history));
    double                          sum = 0.0;
    for (int t = 0; t < trials; ++t)
        for (int d = 0; d < cfg_.history; ++d)
        {
            for (auto& v : blk)
                v = (float)(10.0 * std::log10(g(rng)));
            const float p             = percentileOf(blk, blk.size(), cfg_.percentile);
            est[(size_t)t][(size_t)d] = p;
            sum += p;
        }
    Bias b;
    b.percentile_db = (float)(sum / (trials * cfg_.history));
    b.min_by_d.assign((size_t)cfg_.history + 1, 0.f);
    for (int d = 1; d <= cfg_.history; ++d)
    {
        double s = 0.0;
        for (int t = 0; t < trials; ++t)
        {
            float m = est[(size_t)t][0];
            for (int i = 1; i < d; ++i)
                m = std::min(m, est[(size_t)t][(size_t)i]);
            s += m - b.percentile_db;
        }
        b.min_by_d[(size_t)d] = (float)(s / trials);
    }
    b.min_db = b.min_by_d.back();
    return bias_cache_.emplace(key, std::move(b)).first->second;
}

NoiseFloorResult DspNoiseFloor::estimate(const NoiseFloorInput& in)
{
    NoiseFloorResult r;
    const int        n = in.end_bin - in.start_bin + 1;
    if (!in.power_db || n < 8 || in.start_bin < 0 || in.end_bin >= in.fft_size)
        return r;
    const int M = std::max(1, in.n_frames);
    scratch_.resize((size_t)n);

    // Band-wide ordered statistic (reference for the block clamp).
    std::copy(in.power_db + in.start_bin, in.power_db + in.end_bin + 1, scratch_.begin());
    const float band_raw = percentileOf(scratch_, (size_t)n, cfg_.percentile);
    const float band     = band_raw - bias(M, n).percentile_db;

    // Per-block ordered statistic, bias-corrected to the noise mean.
    const int          n_blocks = std::max(1, n / cfg_.block_bins);
    const int          bsz      = n / n_blocks;  // last block takes the remainder
    std::vector<float> blk_floor((size_t)n_blocks);
    std::vector<float> blk_center((size_t)n_blocks);
    for (int b = 0; b < n_blocks; ++b)
    {
        const int lo = in.start_bin + b * bsz;
        const int hi = (b == n_blocks - 1) ? in.end_bin : lo + bsz - 1;
        const int m  = hi - lo + 1;
        std::copy(in.power_db + lo, in.power_db + hi + 1, scratch_.begin());
        blk_floor[(size_t)b] =
            percentileOf(scratch_, (size_t)m, cfg_.percentile) - bias(M, m).percentile_db;
        blk_center[(size_t)b] = 0.5f * (float)(lo + hi);
    }

    // Minimum statistics over the last `history` sweeps of this entry — for
    // every block and for the band statistic the blocks are clamped to (a
    // burst over most of the band moves that one too).
    float band_ref = band;
    if (cfg_.history > 1)
    {
        auto& hist = history_[in.key];
        if ((int)hist.size() != n_blocks + 1)
            hist.assign((size_t)n_blocks + 1, {});
        auto track = [&](std::vector<float>& h, float v, int m)
        {
            h.push_back(v);
            if ((int)h.size() > cfg_.history)
                h.erase(h.begin());
            const float mn = *std::min_element(h.begin(), h.end());
            // E[min of d unbiased estimates] sits below the mean: undo it.
            return mn - bias(M, m).min_by_d[h.size()];
        };
        for (int b = 0; b < n_blocks; ++b)
            blk_floor[(size_t)b] = track(hist[(size_t)b], blk_floor[(size_t)b], bsz);
        band_ref = track(hist[(size_t)n_blocks], band, n);
    }

    // Clamp around the band statistic, then interpolate to every bin.
    for (auto& f : blk_floor)
        f = std::clamp(f, band_ref - cfg_.max_tilt_db, band_ref + cfg_.max_tilt_db);
    r.floor_db.assign((size_t)in.fft_size, band_ref);
    for (int k = in.start_bin; k <= in.end_bin; ++k)
    {
        const float x = (float)k;
        if (n_blocks == 1 || x <= blk_center.front())
            r.floor_db[(size_t)k] = blk_floor.front();
        else if (x >= blk_center.back())
            r.floor_db[(size_t)k] = blk_floor.back();
        else
        {
            const int   b = std::min(n_blocks - 2, (int)((x - blk_center[0]) / (float)bsz));
            const float t =
                (x - blk_center[(size_t)b]) / (blk_center[(size_t)b + 1] - blk_center[(size_t)b]);
            r.floor_db[(size_t)k] =
                blk_floor[(size_t)b] + t * (blk_floor[(size_t)b + 1] - blk_floor[(size_t)b]);
        }
    }
    std::copy(
        r.floor_db.begin() + in.start_bin, r.floor_db.begin() + in.end_bin + 1, scratch_.begin());
    r.band_floor_db = percentileOf(scratch_, (size_t)n, 0.5);
    r.margin_db     = noiseMarginDb(M, cfg_.pfa);
    r.ok            = true;
    return r;
}

std::unique_ptr<NoiseFloorEstimator> makeNoiseFloorEstimator(const std::string&         method,
                                                             const std::string&         model_path,
                                                             const DspNoiseFloorConfig& dsp)
{
    if (method == "ml")  // planned (ONNX model); not implemented yet
        spdlog::warn("[NoiseFloor] method 'ml' is not available yet — using the DSP estimator "
                     "(model '{}' ignored)",
                     model_path);
    return std::make_unique<DspNoiseFloor>(dsp);
}

}  // namespace acq

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
