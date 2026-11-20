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
// Packet → dwell rules for the SdrResourceManager IQ stream (ICD §10) and the
// scan-request dwell contract with the controller. No broker, no radio.
#include "DwellAssembler.hpp"
#include "SweepConfig.hpp"

#include <gtest/gtest.h>
#include <sdr/Types.hpp>

#include <complex>
#include <cstring>
#include <vector>

using namespace acq;

namespace
{
// Build one ICD packet: header + n samples whose real part is `value`.
std::vector<uint8_t>
pkt(uint32_t seq, uint64_t cf, int n, float value, uint8_t flags = 0, uint8_t ch = 0)
{
    sdr::IqPacketHeader h{};
    h.magic          = sdr::IQ_PACKET_MAGIC;
    h.sequence       = seq;
    h.center_freq_hz = cf;
    h.sample_rate    = 1'000'000;
    h.num_samples    = (uint16_t)n;
    h.channel_index  = ch;
    h.flags          = flags;
    std::vector<uint8_t> b(sizeof h + (size_t)n * 8);
    std::memcpy(b.data(), &h, sizeof h);
    auto* s = reinterpret_cast<std::complex<float>*>(b.data() + sizeof h);
    for (int i = 0; i < n; ++i)
        s[i] = {value, 0.f};
    return b;
}

bool feed(DwellAssembler& a, const std::vector<uint8_t>& b, Dwell& d)
{
    return a.push(b.data(), b.size(), d);
}
}  // namespace

TEST(DwellAssembler, CompletesAtDwellSamples)
{
    DwellAssembler a(1, 300);
    Dwell          d;
    EXPECT_FALSE(feed(a, pkt(0, 100'000'000, 128, 1.f, sdr::IQ_FLAG_FIRST_PACKET), d));
    EXPECT_FALSE(feed(a, pkt(1, 100'000'000, 128, 1.f), d));
    ASSERT_TRUE(feed(a, pkt(2, 100'000'000, 128, 1.f), d));
    EXPECT_EQ(d.ch_samples[0].size(), 300u);  // exactly dwell_samples (ACQ-04)
    EXPECT_DOUBLE_EQ(d.center_hz.in(au::hertz), 100e6);
}

TEST(DwellAssembler, DwellChangeEmitsPreviousDwellAtItsOwnFrequency)
{
    DwellAssembler a(1, 10'000);
    Dwell          d;
    feed(a, pkt(0, 100'000'000, 128, 1.f, sdr::IQ_FLAG_FIRST_PACKET), d);
    feed(a, pkt(1, 100'000'000, 128, 1.f), d);
    ASSERT_TRUE(feed(a, pkt(2, 108'000'000, 128, 2.f, sdr::IQ_FLAG_DWELL_CHANGE), d));
    EXPECT_DOUBLE_EQ(d.center_hz.in(au::hertz), 100e6);
    ASSERT_EQ(d.ch_samples[0].size(), 256u);
    for (auto& s : d.ch_samples[0])
        EXPECT_EQ(s.real(), 1.f) << "new-dwell samples leaked in";
    EXPECT_DOUBLE_EQ(a.currentCenterHz(), 108e6);
}

// A lost UDP packet must not be spliced into the dwell: the PSD would smear
// the discontinuity into every FFT frame spanning it.
TEST(DwellAssembler, SequenceGapRestartsTheDwell)
{
    DwellAssembler a(1, 256);
    Dwell          d;
    feed(a, pkt(0, 100'000'000, 128, 1.f, sdr::IQ_FLAG_FIRST_PACKET), d);
    EXPECT_FALSE(feed(a, pkt(5, 100'000'000, 128, 2.f), d));  // 4 packets lost
    ASSERT_TRUE(feed(a, pkt(6, 100'000'000, 128, 2.f), d));
    ASSERT_EQ(d.ch_samples[0].size(), 256u);
    for (auto& s : d.ch_samples[0])
        EXPECT_EQ(s.real(), 2.f) << "pre-gap samples spliced in";
    EXPECT_EQ(a.stats().gaps, 1u);
    EXPECT_EQ(a.stats().lost_packets, 4u);
    EXPECT_EQ(a.stats().discarded_samples, 128u);
}

// The controller signals a hardware overflow with a zero-sample packet.
TEST(DwellAssembler, FlagOnlyOverflowPacketIsLegalAndRestartsTheDwell)
{
    DwellAssembler a(1, 256);
    Dwell          d;
    feed(a, pkt(0, 100'000'000, 128, 1.f, sdr::IQ_FLAG_FIRST_PACKET), d);
    EXPECT_FALSE(feed(a, pkt(1, 100'000'000, 0, 0.f, sdr::IQ_FLAG_OVERFLOW), d));
    EXPECT_EQ(a.stats().malformed, 0u) << "overflow notification treated as a bad header";
    EXPECT_EQ(a.stats().overflows, 1u);
    feed(a, pkt(2, 100'000'000, 128, 3.f), d);
    ASSERT_TRUE(feed(a, pkt(3, 100'000'000, 128, 3.f), d));
    for (auto& s : d.ch_samples[0])
        EXPECT_EQ(s.real(), 3.f);
}

TEST(DwellAssembler, LateAndDuplicatePacketsAreDropped)
{
    DwellAssembler a(1, 1000);
    Dwell          d;
    feed(a, pkt(10, 100'000'000, 100, 1.f, sdr::IQ_FLAG_FIRST_PACKET), d);
    feed(a, pkt(11, 100'000'000, 100, 1.f), d);
    feed(a, pkt(11, 100'000'000, 100, 9.f), d);  // duplicate
    feed(a, pkt(9, 100'000'000, 100, 9.f), d);   // late
    EXPECT_EQ(a.stats().late_packets, 2u);
    EXPECT_EQ(a.stats().gaps, 0u);
}

TEST(DwellAssembler, GapOnOneChannelKeepsChannelsAligned)
{
    DwellAssembler a(2, 256);
    Dwell          d;
    feed(a, pkt(0, 100'000'000, 128, 1.f, sdr::IQ_FLAG_FIRST_PACKET, 0), d);
    feed(a, pkt(0, 100'000'000, 128, 1.f, sdr::IQ_FLAG_FIRST_PACKET, 1), d);
    feed(a, pkt(2, 100'000'000, 128, 2.f, 0, 1), d);  // ch1 lost a packet
    feed(a, pkt(1, 100'000'000, 128, 2.f, 0, 0), d);
    feed(a, pkt(3, 100'000'000, 128, 2.f, 0, 1), d);
    ASSERT_TRUE(feed(a, pkt(2, 100'000'000, 128, 2.f, 0, 0), d));
    EXPECT_EQ(d.ch_samples[0].size(), d.ch_samples[1].size()) << "channels mis-aligned";
}

TEST(DwellAssembler, TruncatedAndForeignDatagramsAreCountedNotCrashing)
{
    DwellAssembler a(1, 256);
    Dwell          d;
    auto           b = pkt(0, 100'000'000, 128, 1.f);
    EXPECT_FALSE(a.push(b.data(), b.size() - 8, d));  // truncated payload
    std::vector<uint8_t> junk(64, 0xAB);
    EXPECT_FALSE(a.push(junk.data(), junk.size(), d));  // wrong magic
    EXPECT_FALSE(a.push(junk.data(), 10, d));           // shorter than a header
    EXPECT_EQ(a.stats().malformed, 3u);
}

// ── ACQ-04: one dwell per scan entry ────────────────────────────────────────
// The controller streams the whole dwell_ms (≥ 5 ms ⇒ 50 k samples at 10 MS/s)
// while dwell_samples is typically 4096. The assembler emitted a dwell every
// dwell_samples and again for the tail at DWELL_CHANGE, so one scan entry
// produced ~12 dwells at the same frequency — the persistence filter
// ("≥ 2 hits in consecutive sweeps") was satisfied inside a single sweep.
TEST(DwellAssembler, Acq04_OneDwellPerScanEntry)
{
    DwellAssembler a(1, 256);
    Dwell          d;
    int            emitted = 0;
    emitted += feed(
        a, pkt(0, 100'000'000, 128, 1.f, sdr::IQ_FLAG_FIRST_PACKET | sdr::IQ_FLAG_DWELL_CHANGE), d);
    for (uint32_t s = 1; s < 12; ++s)
        emitted += feed(a, pkt(s, 100'000'000, 128, 1.f), d);
    EXPECT_EQ(emitted, 1) << "one scan entry became " << emitted << " dwells";
    // next entry: the 100 MHz tail must not come out as another (partial) dwell
    Dwell d2;
    bool  tail = feed(a, pkt(12, 108'000'000, 128, 2.f, sdr::IQ_FLAG_DWELL_CHANGE), d2);
    EXPECT_FALSE(tail && d2.center_hz.in(au::hertz) == 100e6)
        << "tail re-emitted at the old frequency";
    ASSERT_TRUE(feed(a, pkt(13, 108'000'000, 128, 2.f), d2));
    EXPECT_DOUBLE_EQ(d2.center_hz.in(au::hertz), 108e6);
    for (auto& s : d2.ch_samples[0])
        EXPECT_EQ(s.real(), 2.f);
}

// The controller sets IQ_FLAG_DWELL_CHANGE on the first packet of *every*
// channel after a retune. Channel 1's flag used to emit a second dwell
// holding only channel 0's first packet.
TEST(DwellAssembler, Acq04_FlagOnEveryChannelIsOneDwellChange)
{
    DwellAssembler a(2, 100'000);
    Dwell          d;
    int            emitted = 0;
    feed(a, pkt(0, 100'000'000, 128, 1.f, sdr::IQ_FLAG_FIRST_PACKET, 0), d);
    feed(a, pkt(0, 100'000'000, 128, 1.f, sdr::IQ_FLAG_FIRST_PACKET, 1), d);
    feed(a, pkt(1, 100'000'000, 128, 1.f, 0, 0), d);
    feed(a, pkt(1, 100'000'000, 128, 1.f, 0, 1), d);
    std::vector<double> cfs;
    if (feed(a, pkt(2, 108'000'000, 128, 2.f, sdr::IQ_FLAG_DWELL_CHANGE, 0), d))
    {
        ++emitted;
        cfs.push_back(d.center_hz.in(au::hertz));
    }
    if (feed(a, pkt(2, 108'000'000, 128, 2.f, sdr::IQ_FLAG_DWELL_CHANGE, 1), d))
    {
        ++emitted;
        cfs.push_back(d.center_hz.in(au::hertz));
    }
    EXPECT_EQ(emitted, 1);
    ASSERT_FALSE(cfs.empty());
    EXPECT_DOUBLE_EQ(cfs[0], 100e6);
}

// Channel 0 filling first must not cut channel 1 short (or push channel 1's
// remainder into the next dwell).
TEST(DwellAssembler, Acq04_ChannelsCompleteTogether)
{
    DwellAssembler a(2, 256);
    Dwell          d;
    bool           done = false;
    feed(a, pkt(0, 100'000'000, 128, 1.f, sdr::IQ_FLAG_FIRST_PACKET, 0), d);
    feed(a, pkt(1, 100'000'000, 128, 1.f, 0, 0), d);
    done |= feed(a, pkt(2, 100'000'000, 128, 1.f, 0, 0), d);  // ch0 has 384
    done |= feed(a, pkt(0, 100'000'000, 128, 5.f, sdr::IQ_FLAG_FIRST_PACKET, 1), d);
    if (!done)
        done = feed(a, pkt(1, 100'000'000, 128, 5.f, 0, 1), d);
    ASSERT_TRUE(done);
    ASSERT_EQ(d.ch_samples.size(), 2u);
    EXPECT_EQ(d.ch_samples[0].size(), 256u);
    EXPECT_EQ(d.ch_samples[1].size(), 256u);
}

// ACQ-05: the dwell carries the delivered rate (packet header), not the
// configured one — a fixed-rate / DDC stream otherwise mislabels every bin.
TEST(DwellAssembler, Acq05_DwellCarriesHeaderSampleRate)
{
    DwellAssembler a(1, 128);
    Dwell          d;
    auto           b = pkt(0, 100'000'000, 128, 1.f, sdr::IQ_FLAG_FIRST_PACKET);
    reinterpret_cast<sdr::IqPacketHeader*>(b.data())->sample_rate = 8'000'000;
    ASSERT_TRUE(a.push(b.data(), b.size(), d));
    EXPECT_DOUBLE_EQ(d.sample_rate_hz, 8e6);
}

// A lost DWELL_CHANGE packet: the centre frequency still changes.
TEST(DwellAssembler, Acq04_CentreChangeWithoutFlagStartsANewDwell)
{
    DwellAssembler a(1, 100'000);
    Dwell          d;
    feed(a, pkt(0, 100'000'000, 128, 1.f, sdr::IQ_FLAG_FIRST_PACKET), d);
    feed(a, pkt(1, 100'000'000, 128, 1.f), d);
    ASSERT_TRUE(feed(a, pkt(3, 108'000'000, 128, 2.f), d));  // seq 2 (flagged) lost
    EXPECT_DOUBLE_EQ(d.center_hz.in(au::hertz), 100e6);
    for (auto& s : d.ch_samples[0])
        EXPECT_EQ(s.real(), 1.f);
}

// ACQ-02: the shipped scanner.xml (4096 samples at 10 MS/s = 0.41 ms) produced
// dwell_ms = 1, which SdrResourceManager rejects (< 5 ms) — no scan ever ran.
TEST(ScanDwell, NeverBelowTheControllersMinimum)
{
    EXPECT_EQ(scanDwellMs(4096, 10e6), kControllerMinDwellMs);
    EXPECT_GE(scanDwellMs(1, 61.44e6), kControllerMinDwellMs);
}

TEST(ScanDwell, RoundsUpSoTheDwellHoldsEnoughSamples)
{
    EXPECT_EQ(scanDwellMs(131'072, 2.4e6), 55);  // 54.6 ms → 55, not 54
    EXPECT_GE(scanDwellMs(131'072, 2.4e6) * 2.4e6 / 1000.0, 131072.0);
}

// ACQ-14: <settle_samples> was parsed and never applied. The first samples
// after every retune (and at stream start) are dropped from the dwell.
TEST(DwellAssembler, SettleSamplesAreDroppedAfterEachRetune)
{
    DwellAssembler a(1, 200, 100);
    Dwell          d;
    // entry 1: 128 transient (value 9) then steady (value 1)
    feed(a, pkt(0, 100'000'000, 100, 9.f, sdr::IQ_FLAG_FIRST_PACKET), d);
    feed(a, pkt(1, 100'000'000, 128, 1.f), d);
    ASSERT_TRUE(feed(a, pkt(2, 100'000'000, 128, 1.f), d));
    ASSERT_EQ(d.ch_samples[0].size(), 200u);
    for (auto& v : d.ch_samples[0])
        ASSERT_EQ(v.real(), 1.f) << "stream-start transient in the dwell";
    // entry 2 after a retune
    feed(a, pkt(3, 108'000'000, 100, 9.f, sdr::IQ_FLAG_DWELL_CHANGE), d);
    feed(a, pkt(4, 108'000'000, 128, 2.f), d);
    ASSERT_TRUE(feed(a, pkt(5, 108'000'000, 128, 2.f), d));
    EXPECT_DOUBLE_EQ(d.center_hz.in(au::hertz), 108e6);
    for (auto& v : d.ch_samples[0])
        ASSERT_EQ(v.real(), 2.f) << "post-retune transient in the dwell";
    EXPECT_EQ(a.stats().settled_samples, 200u);
}

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
