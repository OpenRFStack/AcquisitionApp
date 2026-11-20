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
// DspNoiseFloor: accuracy against the analytic floor of the real FftProcessor
// pipeline (IQ in, Blackman-Harris Welch PSD out), with signals, wideband
// interference, duty-cycled bursts and coloured noise; detection rates vs the
// fixed-threshold CA-CFAR it replaces.
#include "FftProcessor.hpp"
#include "NoiseFloorEstimator.hpp"

#include <fftw3.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <random>
#include <vector>

#include <au/units/hertz.hh>

using namespace acq;
using cvec = std::vector<std::complex<float>>;

namespace
{
constexpr int    N  = 4096;
constexpr double FS = 10e6;

// Expected noise level per bin (dBFS) of FftProcessor for complex white
// noise of total variance s2: s2 · Σw² / (Σw)² with its Blackman-Harris window.
double whiteFloorDb(double s2)
{
    double sw = 0, sw2 = 0;
    for (int i = 0; i < N; ++i)
    {
        const double x = 2.0 * M_PI * i / (N - 1);
        const double w =
            0.35875 - 0.48829 * std::cos(x) + 0.14128 * std::cos(2 * x) - 0.01168 * std::cos(3 * x);
        sw += w;
        sw2 += w * w;
    }
    return 10.0 * std::log10(s2 * sw2 / (sw * sw));
}

struct Gen
{
    std::mt19937 rng{7};

    cvec noise(int n, double s2, double tilt_a = 0.0)
    {
        std::normal_distribution<float> g(0.f, (float)std::sqrt(s2 / 2.0));
        cvec                            x((size_t)n + 1);
        for (auto& v : x)
            v = {g(rng), g(rng)};
        cvec y((size_t)n);
        for (int i = 0; i < n; ++i)  // FIR [1, a]: |H|² = 1 + a² + 2a·cos(2πf)
            y[(size_t)i] = x[(size_t)i + 1] + (float)tilt_a * x[(size_t)i];
        return y;
    }

    // Flat band-limited emission [f_lo, f_hi] (fraction of fs, −0.5..0.5) of
    // per-bin level `db_over` relative to a floor of variance s2: synthesised
    // in the frequency domain (random phase, flat magnitude) and inverse-FFTd.
    void addBand(cvec& x, double f_lo, double f_hi, double s2, double db_over)
    {
        const int      n   = (int)x.size();
        fftwf_complex* buf = fftwf_alloc_complex((size_t)n);
        fftwf_plan     p   = fftwf_plan_dft_1d(n, buf, buf, FFTW_BACKWARD, FFTW_ESTIMATE);
        std::uniform_real_distribution<float> ph(0.f, 2.f * (float)M_PI);
        // per-sample variance of the band = s2 · 10^(db/10) · bandwidth
        const double var = s2 * std::pow(10.0, db_over / 10.0) * (f_hi - f_lo);
        int          nb  = 0;
        for (int k = 0; k < n; ++k)
        {
            double f = (double)k / n;
            if (f >= 0.5)
                f -= 1.0;
            const bool in = f >= f_lo && f <= f_hi;
            nb += in;
            const float a = in ? 1.f : 0.f, t = ph(rng);
            buf[k][0] = a * std::cos(t);
            buf[k][1] = a * std::sin(t);
        }
        fftwf_execute(p);
        const float sc = nb > 0 ? (float)std::sqrt(var / nb) : 0.f;  // Parseval
        for (int i = 0; i < n; ++i)
            x[(size_t)i] += std::complex<float>(buf[i][0], buf[i][1]) * sc;
        fftwf_destroy_plan(p);
        fftwf_free(buf);
    }

    void addTone(cvec& x, double f, double s2, double db_over_bin)
    {
        // a tone of power P reads P dBFS in its bin; floor per bin = whiteFloorDb(s2)
        const double p = std::pow(10.0, (whiteFloorDb(s2) + db_over_bin) / 10.0);
        const float  a = (float)std::sqrt(p);
        for (size_t i = 0; i < x.size(); ++i)
            x[i] += std::polar(a, (float)(2 * M_PI * f * (double)i));
    }
};

constexpr float USABLE = 0.8f;

int startBin()
{
    return (int)(N * (1.0 - USABLE) / 2.0);
}

int endBin()
{
    return N - 1 - startBin();
}

NoiseFloorInput inputOf(const FftProcessor& p, uint64_t key = 1)
{
    NoiseFloorInput in;
    in.power_db  = p.powerDb().data();
    in.fft_size  = N;
    in.start_bin = startBin();
    in.end_bin   = endBin();
    in.n_frames  = p.lastFrames();
    in.key       = key;
    return in;
}

float medianOf(std::vector<float> v)
{
    std::nth_element(v.begin(), v.begin() + (ptrdiff_t)v.size() / 2, v.end());
    return v[v.size() / 2];
}

float legacyMedianDb(const FftProcessor& p)
{
    return medianOf(
        std::vector<float>(p.powerDb().begin() + startBin(), p.powerDb().begin() + endBin() + 1));
}
}  // namespace

TEST(NoiseFloor, MarginFollowsTheFrameCount)
{
    // M = 1: exponential bins, P(X > x) = e^−x  →  x = ln(1/pfa)
    EXPECT_NEAR(noiseMarginDb(1, 1e-4), 10.0 * std::log10(std::log(1e4)), 0.01);
    EXPECT_GT(noiseMarginDb(1, 1e-4), noiseMarginDb(7, 1e-4));
    EXPECT_GT(noiseMarginDb(7, 1e-4), noiseMarginDb(31, 1e-4));
    EXPECT_LT(noiseMarginDb(31, 1e-4), 3.5f);
}

// White noise: the estimate is unbiased at every dwell length; the legacy
// median sat 1.6 dB low for one frame (median of an exponential).
TEST(NoiseFloor, WhiteNoiseFloorIsUnbiased)
{
    const double s2   = 1e-4;
    const double want = whiteFloorDb(s2);
    for (int dwell : {4096, 16'384, 65'536})
    {
        Gen           gen;
        FftProcessor  p(N);
        DspNoiseFloor est;
        double        err = 0, err_legacy = 0;
        const int     T = 12;
        for (int t = 0; t < T; ++t)
        {
            auto x = gen.noise(dwell, s2);
            p.computeSpectrum(x.data(), dwell);
            auto r = est.estimate(inputOf(p, (uint64_t)dwell));
            ASSERT_TRUE(r.ok);
            err += r.band_floor_db - want;
            err_legacy += legacyMedianDb(p) - want;
        }
        std::printf(
            "NF white  dwell %6d (%2d frames): DSP error %+.2f dB, legacy median %+.2f dB\n",
            dwell,
            p.lastFrames(),
            err / T,
            err_legacy / T);
        EXPECT_NEAR(err / T, 0.0, 0.5) << "dwell " << dwell;
    }
}

// Wideband interference over 60 % of the examined band, 20 dB above the
// floor: the floor estimate must not follow it.
TEST(NoiseFloor, WidebandInterferenceDoesNotRaiseTheFloor)
{
    const double  s2   = 1e-4;
    const double  want = whiteFloorDb(s2);
    Gen           gen;
    FftProcessor  p(N);
    DspNoiseFloor est;
    auto          x = gen.noise(65'536, s2);
    gen.addBand(x, -0.38, 0.10, s2, 20.0);  // 0.48 of the 0.8 fs examined = 60 %
    p.computeSpectrum(x.data(), (int)x.size());
    auto r = est.estimate(inputOf(p));
    // floor in the quiet part of the band
    std::vector<float> quiet;
    for (int k = startBin(); k <= endBin(); ++k)
    {
        const double f = (double)(k - N / 2) / N;
        if (f > 0.12)
            quiet.push_back(r.floor_db[(size_t)k]);
    }
    const float q = medianOf(quiet);
    std::printf(
        "NF wideband 60%% of band +20 dB: DSP quiet-part floor %+.2f dB, band %+.2f dB, legacy "
        "median %+.2f dB\n",
        q - want,
        r.band_floor_db - want,
        legacyMedianDb(p) - want);
    EXPECT_NEAR(q, want, 1.0);
    EXPECT_GT(legacyMedianDb(p) - want, 10.0) << "scenario too weak to test anything";
}

// A burst covering 80 % of the band in every other sweep: minimum statistics
// over the sweep history keep the floor where it is.
TEST(NoiseFloor, DutyCycledBurstIsIgnoredAcrossSweeps)
{
    const double  s2   = 1e-4;
    const double  want = whiteFloorDb(s2);
    Gen           gen;
    FftProcessor  p(N);
    DspNoiseFloor est;
    float         worst = 0.f;
    for (int sweep = 0; sweep < 16; ++sweep)
    {
        auto x = gen.noise(16'384, s2);
        if (sweep % 2)
            gen.addBand(x, -0.39, 0.25, s2, 15.0);
        p.computeSpectrum(x.data(), (int)x.size());
        auto r = est.estimate(inputOf(p, 42));
        if (sweep >= 2 && (sweep % 2))
            worst = std::max(worst, std::abs(r.band_floor_db - (float)want));
    }
    std::printf("NF duty-cycled 80%% of band +15 dB, every 2nd sweep: worst floor error %.2f dB\n",
                worst);
    EXPECT_LT(worst, 1.0f);
}

// Coloured noise (≈ 5 dB tilt across the band): the floor follows it.
TEST(NoiseFloor, ColouredNoiseIsFollowed)
{
    const double  s2 = 1e-4, a = 0.3;
    Gen           gen;
    FftProcessor  p(N);
    DspNoiseFloor est;
    auto          x = gen.noise(65'536, s2, a);
    p.computeSpectrum(x.data(), (int)x.size());
    auto  r     = est.estimate(inputOf(p));
    float worst = 0.f;
    for (int k = startBin() + 64; k <= endBin() - 64; ++k)
    {
        const double f = (double)(k - N / 2) / N;
        const double want =
            whiteFloorDb(s2) + 10 * std::log10(1 + a * a + 2 * a * std::cos(2 * M_PI * f));
        worst = std::max(worst, std::abs(r.floor_db[(size_t)k] - (float)want));
    }
    std::printf("NF coloured (5 dB tilt): worst per-bin floor error %.2f dB\n", worst);
    EXPECT_LT(worst, 1.5f);
}

// False alarms and weak-carrier detection, DSP floor + analytic margin vs the
// fixed 10 dB CA-CFAR, at three dwell lengths.
TEST(NoiseFloor, DetectionKeepsFalseAlarmsAndGainsSensitivity)
{
    const double s2  = 1e-4;
    auto         run = [&](bool dsp, int dwell, double tone_db, int trials, int& fa, int& hit)
    {
        Gen           gen;
        FftProcessor  p(N);
        DspNoiseFloor est;
        fa = hit = 0;
        for (int t = 0; t < trials; ++t)
        {
            auto         x  = gen.noise(dwell, s2);
            const double ft = 0.13 + t * 1e-5;
            if (tone_db > 0)
                gen.addTone(x, ft, s2, tone_db);
            p.computeSpectrum(x.data(), dwell);
            std::vector<FftProcessor::Signal> sig;
            if (dsp)
            {
                auto r = est.estimate(inputOf(p, (uint64_t)t));  // fresh key: no history help
                sig    = p.detectFromSpectrum(r.margin_db,
                                           USABLE,
                                           au::hertz(FS),
                                           au::hertz(2500.0),
                                           au::hertz(50e3),
                                           3.f,
                                           nullptr,
                                           &r.floor_db);
            }
            else
                sig = p.detectFromSpectrum(
                    10.f, USABLE, au::hertz(FS), au::hertz(2500.0), au::hertz(50e3), 3.f);
            for (auto& s : sig)
            {
                const double f =
                    p.binToHz(s.center_bin, au::hertz(FS), au::hertz(0.0)).in(au::hertz) / FS;
                if (tone_db > 0 && std::abs(f - ft) < 2e-4)
                    ++hit;
                else
                    ++fa;
            }
        }
    };
    for (int dwell : {4096, 16'384, 65'536})
    {
        int fa_l, fa_d, h;
        run(false, dwell, 0, 150, fa_l, h);
        run(true, dwell, 0, 150, fa_d, h);
        std::printf("NF dwell %6d: false alarms/dwell legacy %.3f, DSP %.3f;  Pd legacy/DSP:",
                    dwell,
                    fa_l / 150.0,
                    fa_d / 150.0);
        int pd_dsp_9 = 0, pd_l_9 = 0;
        for (double tdb : {6.0, 9.0, 12.0, 15.0})
        {
            int fl, hl, fd, hd;
            run(false, dwell, tdb, 60, fl, hl);
            run(true, dwell, tdb, 60, fd, hd);
            std::printf("  %2.0f dB %3d/%3d%%", tdb, hl * 100 / 60, hd * 100 / 60);
            if (tdb == 9.0)
            {
                pd_dsp_9 = hd;
                pd_l_9   = hl;
            }
        }
        std::printf("\n");
        EXPECT_LE(fa_d / 150.0, 0.1) << "dwell " << dwell;
        if (dwell >= 16'384)
            EXPECT_GT(pd_dsp_9, pd_l_9) << "dwell " << dwell;
    }
}

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
