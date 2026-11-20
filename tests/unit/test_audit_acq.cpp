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
// Audit regressions for SpectrumScanner / FftProcessor (ACQ-05, ACQ-08).
#include "FftProcessor.hpp"
#include "LocalAddress.hpp"
#include "SpectrumScanner.hpp"
#include "SweepConfig.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <set>
#include <thread>

#include <au/units/hertz.hh>

using namespace acq;

namespace
{
// Delivers the same dwell over and over (like a repeating one-entry scan).
class ScriptedSource : public IqSource
{
public:
    explicit ScriptedSource(Dwell d)
        : d_(std::move(d))
    {
    }

    void open() override
    {
        running_ = true;
    }

    void close() override
    {
    }

    void stop() override
    {
        running_ = false;
    }

    bool next(Dwell& d) override
    {
        if (!running_)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        d = d_;
        return running_;
    }

    int numChannels() const override
    {
        return 1;
    }

private:
    Dwell             d_;
    std::atomic<bool> running_{false};
};

std::vector<std::complex<float>> tone(int n, double f_over_fs, float amp = 1.f)
{
    std::vector<std::complex<float>> v((size_t)n);
    for (int i = 0; i < n; ++i)
        v[(size_t)i] = std::polar(amp, (float)(2 * M_PI * f_over_fs * i));
    return v;
}
}  // namespace

// ACQ-05: the RM delivers 8 MS/s (fixed-rate device / DDC) while the config
// says 1 MS/s. Frequencies must follow the delivered rate.
TEST(AuditAcq, Acq05_DetectionFrequencyUsesDeliveredRate)
{
    SweepConfig cfg;
    cfg.device.rx_channels     = 1;
    cfg.device.sample_rate     = au::hertz(1e6);
    cfg.sweep.start_hz         = au::hertz(96e6);
    cfg.sweep.stop_hz          = au::hertz(104e6);
    cfg.sweep.fft_size         = 1024;
    cfg.sweep.dwell_samples    = 8192;
    cfg.sweep.threshold_db     = 15.0;
    cfg.sweep.min_signal_bw_hz = au::hertz(1000.0);
    Dwell d;
    d.center_hz      = au::hertz(100e6);
    d.sample_rate_hz = 8e6;
    d.ch_samples     = {tone(8192, 1e6 / 8e6)};  // +1 MHz
    ScriptedSource  src(d);
    std::mutex      mu;
    double          fc = 0.0;
    SpectrumScanner sc(cfg,
                       &src,
                       [&](const Detection& det)
                       {
                           std::lock_guard lk(mu);
                           if (fc == 0.0)
                               fc = det.center_freq_hz.in(au::hertz);
                       });
    sc.start();
    for (int i = 0; i < 200; ++i)
    {
        {
            std::lock_guard lk(mu);
            if (fc != 0.0)
                break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    sc.stop();
    EXPECT_NEAR(fc, 101e6, 10e3);
}

// ACQ-08: |X|² was divided by N², ignoring the Blackman-Harris coherent gain
// (Σw ≈ 0.359·N): a full-scale tone read ≈ −8.9 dBFS.
TEST(AuditAcq, Acq08_FullScaleToneReadsZeroDbfs)
{
    FftProcessor p(1024);
    auto         x = tone(8192, 128.0 / 1024.0);  // bin-centred
    p.computeSpectrum(x.data(), (int)x.size());
    float peak = -1e9f;
    for (float v : p.powerDb())
        peak = std::max(peak, v);
    EXPECT_NEAR(peak, 0.0f, 0.1f);
}

// ACQ-06: resuming after a preemption keeps the entry grid. The old rule
// (packet centre + step) put the next task half a step off the grid.
TEST(AuditAcq, Acq06_ResumeKeepsTheEntryGrid)
{
    const double           start = 100e6, stop = 200e6, step = 8e6;
    const auto             fresh = scanEntryCenters(start, stop, step, start);
    const std::set<double> grid(fresh.begin(), fresh.end());
    ASSERT_EQ(fresh.size(), 13u);  // k = 0..12 (start + 12.5·step == stop)
    for (double done : {fresh[0], fresh[4], fresh[12]})
    {
        const double resume = resumeAfterDwell(start, stop, step, done);
        const auto   next   = scanEntryCenters(start, stop, step, resume);
        EXPECT_EQ(std::set<double>(next.begin(), next.end()), grid) << "after " << done;
        // continues with the entry after the completed one
        auto         it     = std::find(fresh.begin(), fresh.end(), done);
        const double expect = (it + 1 == fresh.end()) ? fresh[0] : *(it + 1);
        EXPECT_DOUBLE_EQ(next.front(), expect);
    }
    // The old per-packet rule, for the record: off the grid.
    const double old_resume = fresh[4] + step;
    EXPECT_FALSE(grid.count(old_resume + step / 2.0));
}

// P25 voice channel: a 12.5 kHz span used to produce zero entries, which the
// controller rejects.
TEST(AuditAcq, Acq09_NarrowSpanYieldsOneCentredEntry)
{
    const auto c = scanEntryCenters(851.00625e6 - 6250, 851.00625e6 + 6250, 8e6, 0.0);
    ASSERT_EQ(c.size(), 1u);
    EXPECT_DOUBLE_EQ(c[0], 851.00625e6);
}

// ACQ-07: 0.0.0.0 is not a destination address.
TEST(AuditAcq, Acq07_DestinationForWildcardBind)
{
    EXPECT_EQ(urlHost("amqp://user:pw@broker.lan:5672/vhost"), "broker.lan");
    EXPECT_EQ(urlHost("amqp://10.1.2.3:5672"), "10.1.2.3");
    EXPECT_EQ(urlHost("[::1]:5672"), "::1");
    EXPECT_EQ(localAddressTowards("amqp://127.0.0.1:5672"), "127.0.0.1");
    EXPECT_EQ(localAddressTowards("amqp://no-such-host.invalid:5672", "fb"), "fb");
}

namespace
{
// Band-limited noise: white complex Gaussian (seeded) filtered by an ideal
// brick-wall in the frequency domain — a flat-topped wideband emission.
std::vector<std::complex<float>>
wideband(int n, double f0_over_fs, double bw_over_fs, float snr_db, unsigned seed = 1)
{
    std::vector<std::complex<float>> v((size_t)n);
    uint32_t                         st = seed;
    auto                             u  = [&]
    {
        st = st * 1'664'525u + 1'013'904'223u;
        return ((st >> 8) + 0.5f) / 16777216.f;
    };
    auto g = [&]
    {
        const float r = std::sqrt(-2.f * std::log(u())), t = 2.f * (float)M_PI * u();
        return std::complex<float>(r * std::cos(t), r * std::sin(t));
    };
    // signal = sum of many random-phase tones across the band (flat PSD)
    const int           K   = 400;
    const float         amp = std::pow(10.f, snr_db / 20.f) * 0.01f / std::sqrt((float)K);
    std::vector<double> f(K), ph(K);
    for (int k = 0; k < K; ++k)
    {
        f[(size_t)k]  = f0_over_fs + bw_over_fs * ((k + 0.5) / K - 0.5);
        ph[(size_t)k] = 2 * M_PI * u();
    }
    for (int i = 0; i < n; ++i)
    {
        std::complex<float> x = g() * 0.01f * 0.70710678f;  // noise floor
        for (int k = 0; k < K; ++k)
            x += std::polar(amp, (float)(2 * M_PI * f[(size_t)k] * i + ph[(size_t)k]));
        v[(size_t)i] = x;
    }
    return v;
}
}  // namespace

// ACQ-11: CA-CFAR's reference cells sat inside any emission wider than
// guard + reference bins (40 bins = 98 kHz at 10 MS/s / 4096): the interior
// measured against itself, so a 200 kHz signal came out as edge fragments.
TEST(AuditAcq, Acq11_WideEmissionIsOneDetection)
{
    const double fs = 10e6;
    FftProcessor p(4096, 8, 32);
    auto         x   = wideband(65'536, 2e6 / fs, 200e3 / fs, 20.f);
    auto         sig = p.detect(
        x.data(), (int)x.size(), 10.f, 0.8f, au::hertz(fs), au::hertz(2500.0), au::hertz(50'000.0));
    std::printf("ACQ-11: %zu detection(s)\n", sig.size());
    for (auto& s : sig)
        std::printf("   %.1f kHz .. %.1f kHz (centre %.1f kHz)\n",
                    p.binToHz(s.start_bin, au::hertz(fs), au::hertz(0.0)).in(au::hertz) / 1e3,
                    p.binToHz(s.end_bin, au::hertz(fs), au::hertz(0.0)).in(au::hertz) / 1e3,
                    p.binToHz(s.center_bin, au::hertz(fs), au::hertz(0.0)).in(au::hertz) / 1e3);
    ASSERT_EQ(sig.size(), 1u) << "wide emission split into fragments";
    const double lo = p.binToHz(sig[0].start_bin, au::hertz(fs), au::hertz(0.0)).in(au::hertz);
    const double hi = p.binToHz(sig[0].end_bin, au::hertz(fs), au::hertz(0.0)).in(au::hertz);
    EXPECT_NEAR(hi - lo, 200e3, 30e3);
    EXPECT_NEAR((hi + lo) / 2, 2e6, 15e3);
    // ACQ-12: the reported centre of a flat-topped emission is its centre.
    const double fc = p.binToHz(sig[0].center_bin, au::hertz(fs), au::hertz(0.0)).in(au::hertz);
    EXPECT_NEAR(fc, 2e6, 5e3);
}

// ACQ-13: a weak carrier on a 10 kHz channel raster, jittering ±300 Hz
// between sweeps, alternated between two persistence buckets (floor(fc/10k)),
// so it needed twice the sweeps to confirm.
namespace
{
class AlternatingSource : public IqSource
{
public:
    AlternatingSource(Dwell a, Dwell b)
        : a_(std::move(a))
        , b_(std::move(b))
    {
    }

    void open() override
    {
        running_ = true;
    }

    void close() override
    {
    }

    void stop() override
    {
        running_ = false;
    }

    bool next(Dwell& d) override
    {
        if (!running_ || n_ >= 6)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            return running_ && false;
        }
        d = (n_++ % 2) ? b_ : a_;
        return true;
    }

    int numChannels() const override
    {
        return 1;
    }

    std::atomic<int> n_{0};

private:
    Dwell             a_, b_;
    std::atomic<bool> running_{false};
};
}  // namespace

TEST(AuditAcq, Acq13_RasterCarrierConfirmsOnTheSecondSweep)
{
    SweepConfig cfg;
    cfg.device.rx_channels     = 1;
    cfg.device.sample_rate     = au::hertz(1e6);
    cfg.sweep.fft_size         = 1024;
    cfg.sweep.dwell_samples    = 8192;
    cfg.sweep.threshold_db     = 10.0;
    cfg.sweep.min_signal_bw_hz = au::hertz(500.0);
    const double raster        = 100.1e6;  // multiple of 10 kHz
    auto         mk            = [&](double off)
    {
        Dwell d;
        d.center_hz      = au::hertz(100e6);
        d.sample_rate_hz = 1e6;
        d.ch_samples     = {wideband(8192, (raster + off - 100e6) / 1e6, 0.0, 0.f, 7)};
        return d;
    };
    // a pure tone plus noise: wideband() with zero bandwidth and 0 dB "SNR"
    // gives a carrier ~30 dB over the per-bin floor, PAPR < bypass
    AlternatingSource src(mk(-300.0), mk(+300.0));
    std::mutex        mu;
    std::vector<int>  hits_at;  // dwell index of each emitted detection
    SpectrumScanner   sc(cfg,
                       &src,
                       [&](const Detection& det)
                       {
                           std::lock_guard lk(mu);
                           if (std::abs(det.center_freq_hz.in(au::hertz)-raster) < 2e3)
                               hits_at.push_back(src.n_.load());
                       });
    sc.start();
    for (int i = 0; i < 200 && src.n_.load() < 6; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    sc.stop();
    std::lock_guard lk(mu);
    ASSERT_FALSE(hits_at.empty()) << "never confirmed";
    std::printf("ACQ-13: first detection after dwell %d; %zu detections in 6 dwells\n",
                hits_at.front(),
                hits_at.size());
    EXPECT_EQ(hits_at.front(), 2) << "confirmation delayed by bucket splitting";
    EXPECT_EQ(hits_at.size(), 5u);  // dwells 2..6
}

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
