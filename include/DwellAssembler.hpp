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
 * @file DwellAssembler.hpp
 * @brief Turns SdrResourceManager IQ packets (ICD §10) into Dwells.
 *
 * Split out of TaskManagerIqSource::next() so the packet rules are unit
 * testable without an AMQP broker or a radio:
 *
 *  - IQ_FLAG_DWELL_CHANGE starts a new dwell; the previous one is emitted.
 *  - A dwell never splices across a discontinuity. A sequence gap on any
 *    channel (lost UDP packets) or an IQ_FLAG_OVERFLOW packet discards what
 *    was accumulated for the current dwell on *all* channels (keeping the
 *    channels sample-aligned). The Welch PSD over a spliced buffer smears the
 *    gap into every FFT frame that spans it.
 *  - Late / duplicate packets (sequence behind the last one seen) are dropped.
 *  - Flag-only packets (num_samples == 0) are legal — the controller sends
 *    them to signal an overflow. They used to be logged as "bad header".
 *  - One dwell per scan entry (ACQ-04). The controller streams the whole
 *    dwell_ms — typically many times dwell_samples — so once every channel
 *    holds dwell_samples the dwell is emitted and the rest of that entry is
 *    skipped until the next dwell starts. (It used to emit a dwell every
 *    dwell_samples plus the tail: ~12 dwells per entry at the same frequency.)
 *  - A new dwell starts on IQ_FLAG_DWELL_CHANGE from a channel that already
 *    delivered data in the current dwell (the controller flags the first
 *    packet of *every* channel after a retune), or when the centre frequency
 *    changes (the flagged packet may be the one that was lost).
 *  - A dwell is complete when every channel holds dwell_samples; channels are
 *    trimmed to a common length so they stay sample-aligned. A dwell cut short
 *    by the next one is emitted with what it has.
 *  - Dwell::sample_rate_hz comes from the packet header (ACQ-05).
 */
#include "IqSource.hpp"

#include <sdr/Types.hpp>

#include <algorithm>
#include <complex>
#include <cstdint>
#include <cstring>
#include <vector>

#include <au/units/hertz.hh>

namespace acq
{

class DwellAssembler
{
public:
    struct Stats
    {
        uint64_t packets           = 0;  ///< valid IQ packets seen
        uint64_t gaps              = 0;  ///< sequence discontinuities
        uint64_t lost_packets      = 0;  ///< packets missing according to sequence numbers
        uint64_t late_packets      = 0;  ///< reordered / duplicate packets dropped
        uint64_t overflows         = 0;  ///< IQ_FLAG_OVERFLOW packets
        uint64_t discarded_samples = 0;  ///< samples thrown away to avoid splicing
        uint64_t malformed         = 0;  ///< short / bad-magic / truncated datagrams
        uint64_t skipped_samples   = 0;  ///< beyond dwell_samples within one scan entry
    };

    DwellAssembler(int channels, int dwell_samples)
        : channels_(channels < 1 ? 1 : channels)
        , dwell_samples_(dwell_samples)
    {
        reset();
    }

    void reset()
    {
        clearAccum();
        center_hz_      = 0.0;
        sample_rate_hz_ = 0.0;
        complete_       = false;
        last_seq_.assign((size_t)channels_, -1);
    }

    /// Feed one datagram. Returns true and fills @p out when a dwell completes.
    bool push(const uint8_t* data, size_t n, Dwell& out)
    {
        if (n < sizeof(sdr::IqPacketHeader))
        {
            ++stats_.malformed;
            return false;
        }
        sdr::IqPacketHeader hdr;
        std::memcpy(&hdr, data, sizeof hdr);
        if (hdr.magic != sdr::IQ_PACKET_MAGIC)
        {
            ++stats_.malformed;
            return false;
        }
        const int    ch     = hdr.channel_index;
        const size_t n_samp = hdr.num_samples;
        if (n < sizeof hdr + n_samp * sizeof(std::complex<float>))
        {
            ++stats_.malformed;
            return false;
        }
        ++stats_.packets;
        const bool known_ch = ch >= 0 && ch < channels_;

        bool discontinuity = false;
        if (known_ch)
        {
            int64_t& last = last_seq_[(size_t)ch];
            if (hdr.flags & sdr::IQ_FLAG_FIRST_PACKET)
                last = -1;  // stream (re)start
            if (last >= 0)
            {
                const int32_t diff = (int32_t)(hdr.sequence - (uint32_t)(last + 1));
                if (diff < 0)
                {
                    ++stats_.late_packets;
                    return false;
                }
                if (diff > 0)
                {
                    ++stats_.gaps;
                    stats_.lost_packets += (uint64_t)diff;
                    discontinuity = true;
                }
            }
            last = hdr.sequence;
        }
        if (hdr.flags & sdr::IQ_FLAG_OVERFLOW)
        {
            ++stats_.overflows;
            discontinuity = true;
        }

        const auto*  samples = reinterpret_cast<const std::complex<float>*>(data + sizeof hdr);
        const double cf      = (double)hdr.center_freq_hz;

        // Dwell boundary?
        const bool flag_restart =
            (hdr.flags & sdr::IQ_FLAG_DWELL_CHANGE) && known_ch && seen_[(size_t)ch];
        const bool cf_changed = center_hz_ != 0.0 && cf != center_hz_;
        bool       emitted    = false;
        if (flag_restart || cf_changed)
        {
            if (!complete_ && hasData())
            {
                emit(out);
                emitted = true;
            }
            else
                clearAccum();
            complete_ = false;
        }
        if (center_hz_ == 0.0 || flag_restart || cf_changed)
        {
            center_hz_      = cf;
            sample_rate_hz_ = (double)hdr.sample_rate;
        }
        if (known_ch)
            seen_[(size_t)ch] = true;

        if (complete_)
        {  // rest of an entry already emitted
            stats_.skipped_samples += n_samp;
            return emitted;
        }
        if (discontinuity && hasData())
        {
            for (auto& b : accum_)
                stats_.discarded_samples += b.size();
            clearAccum();
            if (known_ch)
                seen_[(size_t)ch] = true;
        }
        append(ch, samples, n_samp);
        if (!emitted && full())
        {
            emit(out);
            complete_ = true;
            return true;
        }
        return emitted;
    }

    const Stats& stats() const
    {
        return stats_;
    }

    double currentCenterHz() const
    {
        return center_hz_;
    }

private:
    int    channels_;
    int    dwell_samples_;
    double center_hz_      = 0.0;
    double sample_rate_hz_ = 0.0;
    bool   complete_       = false;  ///< current entry's dwell already emitted
    std::vector<std::vector<std::complex<float>>> accum_;
    std::vector<char>    seen_;  ///< channel delivered a packet in this dwell
    std::vector<int64_t> last_seq_;
    Stats                stats_;

    bool hasData() const
    {
        for (auto& b : accum_)
            if (!b.empty())
                return true;
        return false;
    }

    bool full() const
    {
        for (auto& b : accum_)
            if ((int)b.size() < dwell_samples_)
                return false;
        return true;
    }

    void clearAccum()
    {
        accum_.assign((size_t)channels_, {});
        seen_.assign((size_t)channels_, 0);
        for (auto& b : accum_)
            b.reserve((size_t)(dwell_samples_ > 0 ? dwell_samples_ : 0));
    }

    void append(int ch, const std::complex<float>* s, size_t n)
    {
        if (n == 0 || ch < 0 || ch >= channels_)
            return;
        auto&        b    = accum_[(size_t)ch];
        const size_t room = dwell_samples_ > (int)b.size() ? (size_t)dwell_samples_ - b.size() : 0;
        if (n > room)
            stats_.skipped_samples += n - room;
        b.insert(b.end(), s, s + std::min(n, room));
    }

    void emit(Dwell& out)
    {
        // Keep channels sample-aligned: trim to the shortest non-empty channel.
        size_t len = SIZE_MAX;
        for (auto& b : accum_)
            if (!b.empty())
                len = std::min(len, b.size());
        if (len != SIZE_MAX)
            for (auto& b : accum_)
                if (b.size() > len)
                    b.resize(len);
        out.center_hz      = au::hertz(center_hz_);
        out.sample_rate_hz = sample_rate_hz_;
        out.ch_samples     = std::move(accum_);
        clearAccum();
    }
};

}  // namespace acq

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
