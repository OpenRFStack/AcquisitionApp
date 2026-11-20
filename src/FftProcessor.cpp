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
#include "FftProcessor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

#include <au/units/hertz.hh>

namespace acq
{

static constexpr const char* WISDOM_FILE = "/var/cache/sdr-acquisition/fftw_wisdom.txt";

FftProcessor::FftProcessor(int fft_size, int cfar_guard, int cfar_ref)
    : fft_size_(fft_size)
    , cfar_guard_(cfar_guard)
    , cfar_ref_(cfar_ref)
{
    if (fft_size < 64 || (fft_size & (fft_size - 1)) != 0)
        throw std::invalid_argument("fft_size must be a power of 2 >= 64");

    in_  = fftwf_alloc_complex((size_t)fft_size);
    out_ = fftwf_alloc_complex((size_t)fft_size);

    // FFTW_PATIENT benchmarks more candidate algorithms than FFTW_MEASURE and
    // consistently finds faster plans for this size/machine. The planning overhead
    // is paid once and stored in the wisdom file; subsequent starts load instantly.
    fftwf_import_wisdom_from_filename(WISDOM_FILE);
    plan_ = fftwf_plan_dft_1d(
        fft_size, (fftwf_complex*)in_, (fftwf_complex*)out_, FFTW_FORWARD, FFTW_PATIENT);
    fftwf_export_wisdom_to_filename(WISDOM_FILE);

    // Blackman-Harris 4-term: ~92 dB sidelobe rejection.
    window_.resize((size_t)fft_size);
    for (int i = 0; i < fft_size; ++i)
    {
        double x   = 2.0 * M_PI * i / (fft_size - 1);
        window_[i] = 0.35875f - 0.48829f * (float)std::cos(x) + 0.14128f * (float)std::cos(2.0 * x)
                     - 0.01168f * (float)std::cos(3.0 * x);
    }
    // Coherent gain Σw: dividing |X|² by (Σw)² reads a full-scale tone as
    // 0 dBFS. N² (used before) left every level 20·log10(Σw/N) ≈ −8.9 dB low.
    double wsum = 0.0;
    for (float w : window_)
        wsum += w;
    power_norm_ = (float)(1.0 / (wsum * wsum));

    linear_acc_.resize((size_t)fft_size, 0.f);
    power_db_.resize((size_t)fft_size, 0.f);
    scratch_.resize((size_t)fft_size, 0.f);
    psum_.resize((size_t)fft_size + 1, 0.f);  // pre-allocated CFAR prefix sum
    pcnt_.resize((size_t)fft_size + 1, 0);
}

FftProcessor::~FftProcessor()
{
    if (plan_)
        fftwf_destroy_plan(plan_);
    if (in_)
        fftwf_free(in_);
    if (out_)
        fftwf_free(out_);
}

// ── Single frame: DC-remove + window + FFT + accumulate ──────────────────────
void FftProcessor::accumFrame(const std::complex<float>* src)
{
    // Pass 1: DC mean — separate re/im for better auto-vectorization.
    const float* s      = reinterpret_cast<const float*>(src);
    float        sum_re = 0.f, sum_im = 0.f;
    for (int i = 0; i < fft_size_; ++i)
    {
        sum_re += s[2 * i];
        sum_im += s[2 * i + 1];
    }
    const float mean_re = sum_re / (float)fft_size_;
    const float mean_im = sum_im / (float)fft_size_;

    // Pass 2: DC-remove + Blackman-Harris window → FFTW input.
    // __restrict__ lets the compiler emit wider SIMD loads/stores.
    float* __restrict__ c       = reinterpret_cast<float*>(in_);
    const float* __restrict__ w = window_.data();
    for (int i = 0; i < fft_size_; ++i)
    {
        float re     = s[2 * i] - mean_re;
        float im     = s[2 * i + 1] - mean_im;
        float wi     = w[i];
        c[2 * i]     = re * wi;
        c[2 * i + 1] = im * wi;
    }

    fftwf_execute(plan_);

    // fftshift + squared-magnitude accumulation.
    // Two contiguous loops (no modulo) for the best vectorizer hint.
    const float norm             = power_norm_;
    const int   half             = fft_size_ / 2;
    const float* __restrict__ fo = reinterpret_cast<const float*>(out_);
    float* __restrict__ acc      = linear_acc_.data();

    for (int k = 0; k < half; ++k)
    {
        float re = fo[(k + half) * 2];
        float im = fo[(k + half) * 2 + 1];
        acc[k] += (re * re + im * im) * norm;
    }
    for (int k = half; k < fft_size_; ++k)
    {
        float re = fo[(k - half) * 2];
        float im = fo[(k - half) * 2 + 1];
        acc[k] += (re * re + im * im) * norm;
    }
}

// ── Welch averaging ────────────────────────────────────────────────────────────
void FftProcessor::computeSpectrum(const std::complex<float>* samples, int n_samples)
{
    const int hop = fft_size_ / 2;
    // Guard: if fewer samples than one FFT frame, leave the accumulator zeroed.
    // std::max(1,...) would force n_frames=1 and accumFrame would OOB-read.
    if (n_samples < fft_size_)
        return;
    const int n_frames = (n_samples - fft_size_) / hop + 1;
    last_frames_       = n_frames;
    std::fill(linear_acc_.begin(), linear_acc_.end(), 0.f);
    for (int f = 0; f < n_frames; ++f)
        accumFrame(samples + f * hop);

    const float inv_n = 1.f / (float)n_frames;
    for (int k = 0; k < fft_size_; ++k)
        power_db_[k] = 10.f * std::log10(linear_acc_[k] * inv_n + 1e-30f);
}

// ── Noise floor via median (allocation-free) ──────────────────────────────────
float FftProcessor::estimateNoise(int start_bin, int end_bin)
{
    int n = end_bin - start_bin + 1;
    std::copy(power_db_.begin() + start_bin, power_db_.begin() + end_bin + 1, scratch_.begin());
    auto mid = scratch_.begin() + n / 2;
    std::nth_element(scratch_.begin(), mid, scratch_.begin() + n);
    return *mid;
}

// ── CA-CFAR signal detection ──────────────────────────────────────────────────
std::vector<FftProcessor::Signal>
FftProcessor::detectFromSpectrum(float                     threshold_db,
                                 float                     usable_fraction,
                                 au::QuantityD<au::Hertz>  sample_rate,
                                 au::QuantityD<au::Hertz>  min_signal_bw_hz,
                                 au::QuantityD<au::Hertz>  dc_guard_hz,
                                 float                     min_papr_db,
                                 const std::vector<float>* hist_floor,
                                 const std::vector<float>* floor_db)
{
    const double sample_rate_hz = sample_rate.in(au::hertz);
    const double min_bw_hz      = min_signal_bw_hz.in(au::hertz);
    const double dc_guard_raw   = dc_guard_hz.in(au::hertz);

    int margin    = (int)(fft_size_ * (1.0 - usable_fraction) / 2.0);
    int start_bin = margin;
    int end_bin   = fft_size_ - 1 - margin;

    // Blank DC guard zone — suppresses LO leakage sidelobes
    if (dc_guard_raw > 0)
    {
        double bin_hz     = sample_rate_hz / fft_size_;
        int    guard_bins = std::max(1, (int)std::ceil(dc_guard_raw / bin_hz));
        int    center_bin = fft_size_ / 2;
        int    g_lo       = std::max(start_bin, center_bin - guard_bins);
        int    g_hi       = std::min(end_bin, center_bin + guard_bins);
        float  floor_est  = estimateNoise(start_bin, end_bin);
        for (int k = g_lo; k <= g_hi; ++k)
            power_db_[k] = floor_est;
    }

    float global_noise = estimateNoise(start_bin, end_bin);

    // ── Censored CA-CFAR (ACQ-11) ─────────────────────────────────────────
    // Reference cells already `threshold_db` above the band's median floor
    // are signal, not noise: they are left out of the local average. Inside
    // an emission wider than guard + reference cells the plain average
    // measured the emission against itself and nothing was detected. With
    // too few noise-like cells left on both sides, the median floor is used.
    const int   N      = end_bin - start_bin + 1;
    const float censor = global_noise + threshold_db;
    float*      ps     = psum_.data();
    int*        pc     = pcnt_.data();
    ps[0]              = 0.f;
    pc[0]              = 0;
    for (int i = 0; i < N; ++i)
    {
        const float v   = power_db_[start_bin + i];
        const bool  nse = v < censor;
        ps[i + 1]       = ps[i] + (nse ? v : 0.f);
        pc[i + 1]       = pc[i] + (nse ? 1 : 0);
    }

    // {sum, count} of noise-like bins in [a,b] (relative to start_bin).
    auto ref_cells = [&](int a, int b) -> std::pair<float, int>
    {
        a = std::max(a, 0);
        b = std::min(b, N - 1);
        if (a > b)
            return {0.f, 0};
        return {ps[b + 1] - ps[a], pc[b + 1] - pc[a]};
    };

    const int    G        = cfar_guard_;
    const int    R        = cfar_ref_;
    const int    min_ref  = std::max(1, R / 4);  // noise-like cells needed for a local estimate
    const double bin_hz   = sample_rate_hz / fft_size_;
    const int    min_bins = std::max(1, (int)std::ceil(min_bw_hz / bin_hz));

    // ── Run detection with per-bin CA-CFAR threshold ──────────────────────────
    std::vector<Signal> signals;
    int                 run_start    = -1;
    int                 peak_bin_abs = -1;
    float               peak = -1e30f, sum = 0.f;
    int                 count     = 0;
    float               noise_sum = 0.f;               // local noise over the run (flatness gate)
    double              lin_sum = 0.0, lin_mom = 0.0;  // run power centroid

    auto flush = [&](int run_end_abs)
    {
        if (run_start < 0)
            return;
        int width = run_end_abs - run_start + 1;
        if (width >= min_bins)
        {
            float mean = sum / (float)count;
            // The flatness gate is for marginal bumps only: a flat-topped
            // emission (OFDM, LTE, DVB, wideband noise-like modes) well above
            // the floor has peak ≈ mean and used to be dropped (ACQ-11).
            const float run_noise = noise_sum / (float)count;
            const bool  marginal  = mean < run_noise + threshold_db + 3.f;
            if (width < 2 || !marginal || peak - mean >= min_papr_db)
            {
                float center_bin_f = (float)peak_bin_abs;
                // A wide run's peak is just its highest ripple — anywhere in
                // a flat-topped emission (36 kHz off-centre on a 200 kHz one,
                // so the frequency and the persistence key wandered): use the
                // linear-power centroid. Narrow runs: parabolic peak (ACQ-12).
                if (width > 5 && lin_sum > 0.0)
                    center_bin_f = (float)(lin_mom / lin_sum);
                else if (peak_bin_abs > start_bin && peak_bin_abs < end_bin)
                {
                    float y0    = power_db_[peak_bin_abs - 1];
                    float y1    = power_db_[peak_bin_abs];
                    float y2    = power_db_[peak_bin_abs + 1];
                    float denom = 2.0f * (2.f * y1 - y0 - y2);
                    if (std::abs(denom) > 1e-6f)
                        center_bin_f = (float)peak_bin_abs + (y2 - y0) / denom;
                }
                signals.push_back({run_start, run_end_abs, peak, mean, center_bin_f, run_noise});
            }
        }
        run_start    = -1;
        peak_bin_abs = -1;
        peak         = -1e30f;
        sum          = 0.f;
        count        = 0;
        noise_sum    = 0.f;
        lin_sum      = 0.0;
        lin_mom      = 0.0;
    };

    for (int k = start_bin; k <= end_bin; ++k)
    {
        int   i = k - start_bin;
        float local_noise;
        if (floor_db && k < (int)floor_db->size())
            local_noise = (*floor_db)[(size_t)k];  // NoiseFloorEstimator
        else
        {
            const auto [ls, lc] = ref_cells(i - G - R, i - G - 1);
            const auto [rs, rc] = ref_cells(i + G + 1, i + G + R);
            if (lc + rc >= min_ref)
                local_noise = (ls + rs) / (float)(lc + rc);
            else
                local_noise = global_noise;
        }

        if (hist_floor && k < (int)hist_floor->size())
            local_noise = std::max(local_noise, (*hist_floor)[k]);

        if (power_db_[k] >= local_noise + threshold_db)
        {
            if (run_start < 0)
                run_start = k;
            if (power_db_[k] > peak)
            {
                peak         = power_db_[k];
                peak_bin_abs = k;
            }
            sum += power_db_[k];
            noise_sum += local_noise;
            const double lin = std::pow(10.0, (double)power_db_[k] / 10.0);
            lin_sum += lin;
            lin_mom += lin * k;
            ++count;
        }
        else
        {
            flush(k - 1);
        }
    }
    flush(end_bin);

    std::sort(signals.begin(),
              signals.end(),
              [](const Signal& a, const Signal& b)
              {
                  return a.peak_db > b.peak_db;
              });
    return signals;
}

// ── Convenience: spectrum + detect in one call ────────────────────────────────
std::vector<FftProcessor::Signal> FftProcessor::detect(const std::complex<float>* samples,
                                                       int                        n_samples,
                                                       float                      threshold_db,
                                                       float                      usable_fraction,
                                                       au::QuantityD<au::Hertz>   sample_rate,
                                                       au::QuantityD<au::Hertz>   min_signal_bw_hz,
                                                       au::QuantityD<au::Hertz>   dc_guard_hz)
{
    computeSpectrum(samples, n_samples);
    return detectFromSpectrum(
        threshold_db, usable_fraction, sample_rate, min_signal_bw_hz, dc_guard_hz);
}

// ── Frequency conversion ──────────────────────────────────────────────────────
au::QuantityD<au::Hertz> FftProcessor::binToHz(float                    bin,
                                               au::QuantityD<au::Hertz> sample_rate,
                                               au::QuantityD<au::Hertz> center_hz) const
{
    double offset = ((double)bin - fft_size_ / 2) * (sample_rate.in(au::hertz) / fft_size_);
    return au::hertz(center_hz.in(au::hertz) + offset);
}

au::QuantityD<au::Hertz> FftProcessor::binToHz(int                      bin,
                                               au::QuantityD<au::Hertz> sample_rate,
                                               au::QuantityD<au::Hertz> center_hz) const
{
    return binToHz((float)bin, sample_rate, center_hz);
}

}  // namespace acq

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
