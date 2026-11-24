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
#include <random>
#include <set>
#include <thread>
#include <tuple>

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

// ACQ-24: the grid kept only centres <= stop, so the top of the band was
// never scanned: 88-108 MHz at 2.048 MS/s (step 1.6384 MHz) stopped at
// 107.66 MHz. The last window now ends at stop and no centre leaves the band.
TEST(AuditAcq, Acq24_GridCoversTheWholeBand)
{
    for (auto [start, stop, step] : {std::tuple{88e6, 108e6, 1.6384e6},
                                     std::tuple{100e6, 200e6, 8e6},
                                     std::tuple{70e6, 6e9, 8e6},
                                     std::tuple{400e6, 456e6, 8e6}})
    {
        const auto          g = scanEntryCenters(start, stop, step, start);
        std::vector<double> s(g.begin(), g.end());
        std::sort(s.begin(), s.end());
        EXPECT_NEAR(s.front() - step / 2, start, 1.0);
        EXPECT_NEAR(s.back() + step / 2, stop, 1.0) << "top of " << start << "-" << stop;
        for (size_t i = 1; i < s.size(); ++i)
            EXPECT_LE(s[i] - s[i - 1], step + 1.0) << "gap in the coverage";
        for (double c : s)
        {
            EXPECT_GE(c - step / 2, start - 1.0);
            EXPECT_LE(c + step / 2, stop + 1.0);
        }
        // a resume after any entry continues with the next one, every entry once
        for (size_t i = 0; i < g.size(); ++i)
        {
            const auto next =
                scanEntryCenters(start, stop, step, resumeAfterDwell(start, stop, step, g[i]));
            EXPECT_EQ(std::set<double>(next.begin(), next.end()),
                      std::set<double>(g.begin(), g.end()));
            EXPECT_DOUBLE_EQ(next.front(), g[(i + 1) % g.size()]);
        }
    }
    std::printf("ACQ-24: 88-108 MHz top covered %.3f MHz (was 107.661)\n",
                (scanEntryCenters(88e6, 108e6, 1.6384e6, 88e6).back() + 0.8192e6) / 1e6);
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

// ACQ-15: the persistence bypass for strong signals looked at PAPR only; a
// flat-topped emission 37 dB over the floor still waited for a second sweep.
TEST(AuditAcq, Acq15_StrongFlatEmissionEmitsOnTheFirstSweep)
{
    SweepConfig cfg;
    cfg.device.rx_channels     = 1;
    cfg.device.sample_rate     = au::hertz(10e6);
    cfg.sweep.fft_size         = 4096;
    cfg.sweep.min_signal_bw_hz = au::hertz(2500.0);
    Dwell d;
    d.center_hz      = au::hertz(100e6);
    d.sample_rate_hz = 10e6;
    d.ch_samples     = {wideband(32'768, 2e6 / 10e6, 200e3 / 10e6, 20.f, 3)};
    AlternatingSource src(d, d);
    std::mutex        mu;
    std::vector<int>  hits_at;
    double            snr = 0, floor = 0;
    SpectrumScanner   sc(cfg,
                       &src,
                       [&](const Detection& det)
                       {
                           std::lock_guard lk(mu);
                           if (std::abs(det.center_freq_hz.in(au::hertz)-102e6) < 20e3)
                           {
                               hits_at.push_back(src.n_.load());
                               snr   = det.snr_db;
                               floor = det.noise_floor_db;
                           }
                       });
    sc.start();
    for (int i = 0; i < 400 && src.n_.load() < 6; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    sc.stop();
    std::lock_guard lk(mu);
    ASSERT_FALSE(hits_at.empty());
    std::printf("ACQ-15: first detection on dwell %d, SNR %.1f dB, floor %.1f dBFS\n",
                hits_at.front(),
                snr,
                floor);
    EXPECT_EQ(hits_at.front(), 1);
    EXPECT_GT(snr, 25.0);
}

// Noise map: a continuous carrier shows in quiet_db and occupancy at its
// point, while floor_db there stays the surrounding floor — it is reported,
// never absorbed into the floor used for detection.
TEST(AuditAcq, NoiseMapReportsPersistentCarrierWithoutRaisingTheFloor)
{
    SweepConfig cfg;
    cfg.device.rx_channels     = 1;
    cfg.device.sample_rate     = au::hertz(10e6);
    cfg.sweep.fft_size         = 4096;
    cfg.sweep.min_signal_bw_hz = au::hertz(2500.0);
    cfg.noise_map.enabled      = true;
    cfg.noise_map.every_sweeps = 3;
    cfg.noise_map.bins         = 64;
    Dwell d;
    d.center_hz      = au::hertz(100e6);
    d.sample_rate_hz = 10e6;
    d.ch_samples     = {wideband(32'768, 1e6 / 10e6, 0.0, 10.f, 5)};  // carrier at +1 MHz
    AlternatingSource     src(d, d);
    std::mutex            mu;
    std::vector<NoiseMap> maps;
    SpectrumScanner       sc(cfg,
                       &src,
                       [](const Detection&)
                       {
                       });
    sc.setNoiseMapCallback(
        [&](const NoiseMap& m)
        {
            std::lock_guard lk(mu);
            maps.push_back(m);
        });
    sc.start();
    for (int i = 0; i < 400 && src.n_.load() < 6; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    sc.stop();
    std::lock_guard lk(mu);
    ASSERT_EQ(maps.size(), 2u) << "one map per 3 sweeps";
    const auto& m = maps[0];
    EXPECT_EQ(m.sweeps, 3);
    ASSERT_EQ(m.floor_db.size(), 64u);
    const int carrier = (int)((101e6 - m.f_start_hz) / m.point_hz);
    ASSERT_GE(carrier, 0);
    ASSERT_LT(carrier, 64);
    std::vector<float> fl = m.floor_db;
    std::nth_element(fl.begin(), fl.begin() + 32, fl.end());
    const float med_floor = fl[32];
    std::printf(
        "NoiseMap: point %d (101 MHz): floor %.1f, quiet %.1f, occupancy %.2f; median floor "
        "%.1f dBFS\n",
        carrier,
        m.floor_db[(size_t)carrier],
        m.quiet_db[(size_t)carrier],
        m.occupancy[(size_t)carrier],
        med_floor);
    EXPECT_NEAR(m.floor_db[(size_t)carrier], med_floor, 1.0) << "carrier absorbed into the floor";
    EXPECT_GT(m.quiet_db[(size_t)carrier], med_floor + 20.f);
    EXPECT_FLOAT_EQ(m.occupancy[(size_t)carrier], 1.f);
    EXPECT_LT(m.occupancy[(size_t)((carrier + 20) % 64)], 0.5f);
}

// ── Persistence false-alarm rate (ACQ-20) ─────────────────────────────────────
// Fresh complex noise every dwell (one-entry repeating scan at 10 MS/s,
// 4096-pt, 65 536-sample dwells: the production defaults), optionally with a
// weak CW carrier. Reports false detections per dwell and the fraction of
// dwells that report the carrier.
namespace
{
class NoiseSource : public IqSource
{
public:
    NoiseSource(int dwells, double tone_off_hz, float tone_amp)
        : left_(dwells)
        , off_(tone_off_hz)
        , amp_(tone_amp)
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
        if (!running_ || left_ <= 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            return false;
        }
        --left_;
        d.center_hz      = au::hertz(100e6);
        d.sample_rate_hz = 10e6;
        std::vector<std::complex<float>> v(65'536);
        std::normal_distribution<float>  n(0.f, 0.01f);
        const double                     w = 2 * M_PI * off_ / 10e6;
        for (size_t i = 0; i < v.size(); ++i)
        {
            v[i] = {n(g_), n(g_)};
            if (amp_ > 0.f)
            {
                v[i] += std::polar(amp_, (float)std::fmod(phase_, 2 * M_PI));
                phase_ += w;
            }
        }
        d.ch_samples = {std::move(v)};
        ++done_;
        return true;
    }

    int numChannels() const override
    {
        return 1;
    }

    std::atomic<int> done_{0};

private:
    std::atomic<int>  left_;
    double            off_;
    float             amp_;
    double            phase_ = 0.0;
    std::mt19937      g_{4242};
    std::atomic<bool> running_{false};
};

struct PersistStats
{
    double false_per_dwell;
    double carrier_dwell_fraction;
};

PersistStats runPersistence(int dwells, float tone_amp, double min_bw_hz = 500.0)
{
    SweepConfig cfg;
    cfg.device.rx_channels     = 1;
    cfg.device.sample_rate     = au::hertz(10e6);
    cfg.sweep.fft_size         = 4096;
    cfg.sweep.dwell_samples    = 65'536;
    cfg.sweep.min_signal_bw_hz = au::hertz(min_bw_hz);
    const double    tone_off   = 1.234e6;
    NoiseSource     src(dwells, tone_off, tone_amp);
    std::mutex      mu;
    int             false_n = 0;
    std::set<int>   tone_dwells;
    SpectrumScanner sc(cfg,
                       &src,
                       [&](const Detection& det)
                       {
                           std::lock_guard lk(mu);
                           if (tone_amp > 0.f
                               && std::abs(det.center_freq_hz.in(au::hertz) - (100e6 + tone_off))
                                      < 20e3)
                               tone_dwells.insert(src.done_.load());
                           else
                               ++false_n;
                       });
    sc.start();
    for (int i = 0; i < 6000 && src.done_.load() < dwells; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    sc.stop();
    std::lock_guard lk(mu);
    return {(double)false_n / dwells, (double)tone_dwells.size() / dwells};
}
}  // namespace

TEST(AuditAcq, Acq20_PersistenceFalseAlarmRate)
{
    const float levels[] = {0.00024f, 0.00034f, 0.00048f, 0.00085f};
    for (double min_bw : {500.0, 2500.0})  // 1-bin and the production 2-bin minimum
    {
        const auto noise = runPersistence(100, 0.f, min_bw);
        std::printf("ACQ-20 min_bw %.0f Hz: noise only: %.2f false detections per dwell\n",
                    min_bw,
                    noise.false_per_dwell);
        double prev = 0.0;
        for (float a : levels)
        {
            const auto w = runPersistence(100, a, min_bw);
            std::printf("ACQ-20 min_bw %.0f Hz: carrier %.1f dB: reported in %.0f %% of dwells\n",
                        min_bw,
                        20 * std::log10(a / 0.01414f),
                        100 * w.carrier_dwell_fraction);
            // A stronger carrier must not be reported less often (it was:
            // 11 % at -33.5 dB, 1 % at -31 dB with a 1-bin minimum).
            EXPECT_GE(w.carrier_dwell_fraction, prev - 0.10) << "level " << a;
            prev = std::max(prev, w.carrier_dwell_fraction);
            if (min_bw < 1000.0 && a == 0.00034f)  // -32.4 dB: 3 % before ACQ-20
                EXPECT_GE(w.carrier_dwell_fraction, 0.8);
        }
        EXPECT_LT(noise.false_per_dwell, min_bw < 1000.0 ? 0.8 : 0.3);
    }
}

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
