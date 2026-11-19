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
#include <gtest/gtest.h>
#include "SpectrumScanner.hpp"
#include "FftProcessor.hpp"
#include "SweepConfig.hpp"
#include "LocalAddress.hpp"
#include <set>
#include <au/units/hertz.hh>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include <algorithm>

using namespace acq;

namespace {
// Delivers the same dwell over and over (like a repeating one-entry scan).
class ScriptedSource : public IqSource {
public:
    explicit ScriptedSource(Dwell d) : d_(std::move(d)) {}
    void open() override { running_ = true; }
    void close() override {}
    void stop() override { running_ = false; }
    bool next(Dwell& d) override {
        if (!running_) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        d = d_;
        return running_;
    }
    int numChannels() const override { return 1; }
private:
    Dwell d_;
    std::atomic<bool> running_{false};
};

std::vector<std::complex<float>> tone(int n, double f_over_fs, float amp = 1.f) {
    std::vector<std::complex<float>> v((size_t)n);
    for (int i = 0; i < n; ++i)
        v[(size_t)i] = std::polar(amp, (float)(2 * M_PI * f_over_fs * i));
    return v;
}
} // namespace

// ACQ-05: the RM delivers 8 MS/s (fixed-rate device / DDC) while the config
// says 1 MS/s. Frequencies must follow the delivered rate.
TEST(AuditAcq, Acq05_DetectionFrequencyUsesDeliveredRate) {
    SweepConfig cfg;
    cfg.device.rx_channels       = 1;
    cfg.device.sample_rate       = au::hertz(1e6);
    cfg.sweep.start_hz           = au::hertz(96e6);
    cfg.sweep.stop_hz            = au::hertz(104e6);
    cfg.sweep.fft_size           = 1024;
    cfg.sweep.dwell_samples      = 8192;
    cfg.sweep.threshold_db       = 15.0;
    cfg.sweep.min_signal_bw_hz   = au::hertz(1000.0);
    Dwell d;
    d.center_hz      = au::hertz(100e6);
    d.sample_rate_hz = 8e6;
    d.ch_samples     = {tone(8192, 1e6 / 8e6)};       // +1 MHz
    ScriptedSource src(d);
    std::mutex mu;
    double fc = 0.0;
    SpectrumScanner sc(cfg, &src, [&](const Detection& det) {
        std::lock_guard lk(mu);
        if (fc == 0.0) fc = det.center_freq_hz.in(au::hertz);
    });
    sc.start();
    for (int i = 0; i < 200; ++i) {
        { std::lock_guard lk(mu); if (fc != 0.0) break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    sc.stop();
    EXPECT_NEAR(fc, 101e6, 10e3);
}

// ACQ-08: |X|² was divided by N², ignoring the Blackman-Harris coherent gain
// (Σw ≈ 0.359·N): a full-scale tone read ≈ −8.9 dBFS.
TEST(AuditAcq, Acq08_FullScaleToneReadsZeroDbfs) {
    FftProcessor p(1024);
    auto x = tone(8192, 128.0 / 1024.0);               // bin-centred
    p.computeSpectrum(x.data(), (int)x.size());
    float peak = -1e9f;
    for (float v : p.powerDb()) peak = std::max(peak, v);
    EXPECT_NEAR(peak, 0.0f, 0.1f);
}

// ACQ-06: resuming after a preemption keeps the entry grid. The old rule
// (packet centre + step) put the next task half a step off the grid.
TEST(AuditAcq, Acq06_ResumeKeepsTheEntryGrid) {
    const double start = 100e6, stop = 200e6, step = 8e6;
    const auto fresh = scanEntryCenters(start, stop, step, start);
    const std::set<double> grid(fresh.begin(), fresh.end());
    ASSERT_EQ(fresh.size(), 13u);   // k = 0..12 (start + 12.5·step == stop)
    for (double done : {fresh[0], fresh[4], fresh[12]}) {
        const double resume = resumeAfterDwell(start, stop, step, done);
        const auto next = scanEntryCenters(start, stop, step, resume);
        EXPECT_EQ(std::set<double>(next.begin(), next.end()), grid) << "after " << done;
        // continues with the entry after the completed one
        auto it = std::find(fresh.begin(), fresh.end(), done);
        const double expect = (it + 1 == fresh.end()) ? fresh[0] : *(it + 1);
        EXPECT_DOUBLE_EQ(next.front(), expect);
    }
    // The old per-packet rule, for the record: off the grid.
    const double old_resume = fresh[4] + step;
    EXPECT_FALSE(grid.count(old_resume + step / 2.0));
}

// P25 voice channel: a 12.5 kHz span used to produce zero entries, which the
// controller rejects.
TEST(AuditAcq, Acq09_NarrowSpanYieldsOneCentredEntry) {
    const auto c = scanEntryCenters(851.00625e6 - 6250, 851.00625e6 + 6250, 8e6, 0.0);
    ASSERT_EQ(c.size(), 1u);
    EXPECT_DOUBLE_EQ(c[0], 851.00625e6);
}

// ACQ-07: 0.0.0.0 is not a destination address.
TEST(AuditAcq, Acq07_DestinationForWildcardBind) {
    EXPECT_EQ(urlHost("amqp://user:pw@broker.lan:5672/vhost"), "broker.lan");
    EXPECT_EQ(urlHost("amqp://10.1.2.3:5672"), "10.1.2.3");
    EXPECT_EQ(urlHost("[::1]:5672"), "::1");
    EXPECT_EQ(localAddressTowards("amqp://127.0.0.1:5672"), "127.0.0.1");
    EXPECT_EQ(localAddressTowards("amqp://no-such-host.invalid:5672", "fb"), "fb");
}

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
