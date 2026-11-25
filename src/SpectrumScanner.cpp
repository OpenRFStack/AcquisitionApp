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
#include "SpectrumScanner.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>
#include <unordered_set>

#include <au/prefix.hh>
#include <au/units/seconds.hh>

namespace acq
{

using namespace std::chrono;

SpectrumScanner::SpectrumScanner(SweepConfig cfg, IqSource* source, DetectionCallback cb)
    : cfg_(std::move(cfg))
    , source_(source)
    , cb_(std::move(cb))
{
    int n_ch = cfg_.device.rx_channels;
    for (int i = 0; i < n_ch; ++i)
        processors_.push_back(std::make_unique<FftProcessor>(
            cfg_.sweep.fft_size, cfg_.sweep.cfar_guard_bins, cfg_.sweep.cfar_ref_bins));
    DspNoiseFloorConfig nf;
    nf.pfa         = cfg_.sweep.noise_pfa;
    nf.percentile  = cfg_.sweep.noise_percentile;
    nf.block_bins  = cfg_.sweep.noise_block_bins;
    nf.history     = cfg_.sweep.noise_history;
    nf.max_tilt_db = cfg_.sweep.noise_max_tilt_db;
    for (int i = 0; i < n_ch; ++i)
        noise_.push_back(cfg_.sweep.noise_method == "cfar"
                             ? nullptr
                             : makeNoiseFloorEstimator(cfg_.sweep.noise_method, "", nf));
    ch_persist_.resize((size_t)n_ch);
    ch_map_.resize((size_t)n_ch);
}

SpectrumScanner::~SpectrumScanner()
{
    stop();
}

void SpectrumScanner::start()
{
    running_ = true;
    thread_  = std::thread(
        [this]
        {
            sweepLoop();
        });
}

void SpectrumScanner::stop()
{
    running_ = false;
    if (source_)
        source_->stop();
    if (thread_.joinable())
        thread_.join();
}

void SpectrumScanner::sweepLoop()
{
    static constexpr int kMinRetryMs = 500, kMaxRetryMs = 5000;
    int                  retry_ms = kMinRetryMs;
    while (running_)
    {
        try
        {
            source_->open();
        }
        catch (const std::exception& e)
        {
            // open() binds the UDP receive socket (and may submit a task)
            // before it can throw -- a rejected/timed-out submission left
            // that socket (each with a 50MB SO_RCVBUF request) leaked
            // forever, since close() below was only reachable from the
            // success path. Confirmed live: `ss -uanp` showed 7 simultaneous
            // leaked sockets on one sdr_acquisition process after just a
            // few retries. close() is safe to call here even on a
            // never-fully-opened source (sendTaskStop() no-ops without a
            // task_id_, and the fd guard handles udp_fd_ == -1).
            source_->close();
            // Back off 0.5 s doubling to 5 s: a fixed 5 s left the scanner
            // idle long after the radio was free again (a rejection costs the
            // controller a few ms). Reset after the next successful open.
            spdlog::error(
                "[Scanner] source open failed: {} — retrying in {} ms", e.what(), retry_ms);
            for (int waited = 0; waited < retry_ms && running_; waited += 100)
                std::this_thread::sleep_for(milliseconds(100));
            retry_ms = std::min(2 * retry_ms, kMaxRetryMs);
            continue;
        }
        retry_ms = kMinRetryMs;

        spdlog::info("[Scanner] source open: {} channel(s)  {:.3f}–{:.3f} MHz",
                     cfg_.device.rx_channels,
                     cfg_.sweep.start_hz.in(au::hertz) / 1e6,
                     cfg_.sweep.stop_hz.in(au::hertz) / 1e6);

        while (running_)
        {
            Dwell dwell;
            if (!source_->next(dwell) || !running_)
                break;

            int n_ch = std::min(static_cast<int>(dwell.ch_samples.size()),
                                static_cast<int>(processors_.size()));
            // Bin → Hz uses the rate the samples were delivered at (packet
            // header), not the configured one (ACQ-05): a fixed-rate or
            // DDC-decimated stream otherwise mislabels every detection.
            const auto sr = dwell.sample_rate_hz > 0.0 ? au::hertz(dwell.sample_rate_hz)
                                                       : cfg_.device.sample_rate;

            if (n_ch == 1)
            {
                if (dwell.ch_samples[0].size() >= (size_t)cfg_.sweep.fft_size)
                {
                    for (auto& d : processDwell(0, dwell.center_hz, sr, dwell.ch_samples[0]))
                        cb_(d);
                }
            }
            else
            {
                // Multi-channel: process concurrently, fire callbacks on sweep thread.
                std::vector<std::future<std::vector<Detection>>> futs;
                futs.reserve((size_t)n_ch);
                for (int ch = 0; ch < n_ch; ++ch)
                {
                    if (dwell.ch_samples[ch].size() < (size_t)cfg_.sweep.fft_size)
                        continue;
                    futs.push_back(std::async(
                        std::launch::async,
                        [this, ch, &dwell, sr]
                        {
                            return processDwell(ch, dwell.center_hz, sr, dwell.ch_samples[ch]);
                        }));
                }
                for (auto& f : futs)
                    for (auto& d : f.get())
                        cb_(d);
            }
        }

        source_->close();

        if (running_ && cfg_.analysis_pause_ms > au::seconds(0.0))
        {
            auto pause_ms =
                static_cast<long long>(cfg_.analysis_pause_ms.in(au::milli(au::seconds)));
            spdlog::info("[Scanner] analysis window — pausing {}ms before next sweep", pause_ms);
            auto pause_end = steady_clock::now() + milliseconds(pause_ms);
            while (running_ && steady_clock::now() < pause_end)
                std::this_thread::sleep_for(milliseconds(100));
        }
    }
}

std::vector<Detection>
SpectrumScanner::processDwell(int                                     ch,
                              au::QuantityD<au::Hertz>                center_hz,
                              au::QuantityD<au::Hertz>                sample_rate,
                              const std::vector<std::complex<float>>& samples)
{
    auto& proc = *processors_[ch];

    // Use raw Hz value as map key for the persistence and noise-floor maps.
    const uint64_t center_hz_raw = static_cast<uint64_t>(std::llround(center_hz.in(au::hertz)));

    // ── Step 1: Welch spectrum (includes IQ correction estimation) ────────────
    proc.computeSpectrum(samples.data(), static_cast<int>(samples.size()));

    // ── Step 2: noise floor + detection ───────────────────────────────────────
    // NoiseFloorEstimator ("dsp"): per-bin floor and a false-alarm-rate margin
    // that follows the number of Welch frames. "cfar": censored CA-CFAR with
    // the fixed threshold_db.
    NoiseFloorResult nf;
    if (noise_[ch])
    {
        const auto [lo, hi] = proc.usableRange(static_cast<float>(cfg_.sweep.usable_bw_fraction));
        NoiseFloorInput in;
        in.power_db  = proc.powerDb().data();
        in.fft_size  = proc.fft_size();
        in.start_bin = lo;
        in.end_bin   = hi;
        in.n_frames  = proc.lastFrames();
        in.key       = center_hz_raw;
        nf           = noise_[ch]->estimate(in);
    }
    auto sigs =
        proc.detectFromSpectrum(nf.ok ? nf.margin_db : static_cast<float>(cfg_.sweep.threshold_db),
                                static_cast<float>(cfg_.sweep.usable_bw_fraction),
                                sample_rate,
                                cfg_.sweep.min_signal_bw_hz,
                                cfg_.sweep.dc_guard_hz,
                                cfg_.sweep.min_papr_db,
                                nullptr,
                                nf.ok ? &nf.floor_db : nullptr);

    if (map_cb_ && cfg_.noise_map.enabled && nf.ok)
        accumulateNoiseMap(
            ch, center_hz_raw, center_hz.in(au::hertz), sample_rate.in(au::hertz), proc, nf);

    // ── Step 4: Build IQ snapshot for ONNX fast-path / MUSIC ─────────────────
    // Built once per dwell and shared across all Detection objects via shared_ptr
    // (zero-copy: N detections → one allocation instead of N × 8 KB copies).
    static constexpr int SNAP_SAMPLES   = 1024;
    auto                 dwell_snapshot = std::make_shared<std::vector<float>>();
    {
        int snap_n = std::min(SNAP_SAMPLES, (int)samples.size());
        dwell_snapshot->reserve(snap_n * 2);
        for (int i = 0; i < snap_n; ++i)
        {
            dwell_snapshot->push_back(samples[i].real());
            dwell_snapshot->push_back(samples[i].imag());
        }
    }

    // ── Step 5: Persistence filter ────────────────────────────────────────────
    // Each candidate must be detected in ≥ PERSIST_MIN_HITS consecutive sweeps
    // before being emitted. Strong signals (SNR ≥ PERSIST_BYPASS_SNR_DB or
    // PAPR ≥ PERSIST_BYPASS_PAPR_DB) emit
    // immediately without waiting for confirmation — they're clearly real.
    auto&                        pmap = ch_persist_[ch][center_hz_raw];
    std::unordered_set<uint64_t> seen_this_dwell;
    seen_this_dwell.reserve(sigs.size() * 2);

    auto                   now = system_clock::now();
    std::vector<Detection> out;
    out.reserve(sigs.size());

    for (const auto& s : sigs)
    {
        auto     f_lo_q = proc.binToHz(s.start_bin, sample_rate, center_hz);
        auto     f_hi_q = proc.binToHz(s.end_bin, sample_rate, center_hz);
        auto     fc_q   = proc.binToHz(s.center_bin, sample_rate, center_hz);
        uint64_t f_lo   = static_cast<uint64_t>(std::llround(f_lo_q.in(au::hertz)));
        uint64_t f_hi   = static_cast<uint64_t>(std::llround(f_hi_q.in(au::hertz)));
        uint64_t fc     = static_cast<uint64_t>(std::llround(fc_q.in(au::hertz)));
        // Same emission = the nearest entry within ±PERSIST_FREQ_QUANT_HZ.
        // floor(fc / 10 kHz) buckets split every signal on a 10/12.5/25 kHz
        // channel raster in two: sub-bin jitter around the raster frequency
        // alternated between neighbouring buckets (ACQ-13).
        // Only an entry this run overlaps (±1 bin) counts as the same
        // emission: any entry within ±10 kHz used to (±4 bins at 10 MS/s /
        // 4096, ±41 at 1 MS/s), so noise crossings confirmed each other —
        // about one false detection per dwell (ACQ-20).
        const uint64_t pad  = (uint64_t)std::llround(sample_rate.in(au::hertz) / proc.fft_size());
        uint64_t       key  = fc;
        uint64_t       best = PERSIST_FREQ_QUANT_HZ + 1;
        for (const auto& [k, e] : pmap)
        {
            const uint64_t dist    = k > fc ? k - fc : fc - k;
            const bool     overlap = f_lo <= e.hi + pad && f_hi + pad >= e.lo;
            if (overlap && dist < best && !seen_this_dwell.count(k))
            {
                best = dist;
                key  = k;
            }
        }
        if (best > PERSIST_FREQ_QUANT_HZ)
            key = fc;

        seen_this_dwell.insert(key);

        auto& pe = pmap[key];
        ++pe.hits;
        pe.misses  = 0;
        pe.lo      = f_lo;
        pe.hi      = f_hi;
        pe.peak_db = std::max(pe.peak_db, s.peak_db);

        // Strong = well above the floor (FM broadcast, LTE …). It used to be
        // judged by PAPR alone, which a flat-topped emission never reaches at
        // any level, so a strong LTE carrier waited for confirmation (ACQ-15).
        bool strong = (s.peak_db - s.noise_db) >= PERSIST_BYPASS_SNR_DB
                      || (s.peak_db - s.mean_db) >= PERSIST_BYPASS_PAPR_DB;
        bool confirm = pe.hits >= PERSIST_MIN_HITS;
        if (!strong && !confirm)
            continue;  // first-hit noise candidate: skip

        spdlog::debug("[Scanner] ch{} {:.3f} MHz  BW {:.1f} kHz  {:.1f} dB"
                      "  PAPR {:.1f} dB  hits={}",
                      ch,
                      fc / 1e6,
                      (f_hi > f_lo ? f_hi - f_lo : 0) / 1e3,
                      s.peak_db,
                      s.peak_db - s.mean_db,
                      pe.hits);

        Detection d;
        d.timestamp      = now;
        d.center_freq_hz = fc_q;
        d.bandwidth_hz   = au::hertz(f_hi > f_lo ? static_cast<double>(f_hi - f_lo) : 1.0);
        d.power_db       = s.peak_db;
        d.scanner_id     = cfg_.scanner_id;
        d.channel        = ch;
        d.snr_db         = s.peak_db - s.noise_db;  // above the local floor
        d.papr_db        = s.peak_db - s.mean_db;
        d.noise_floor_db = s.noise_db;
        if (cfg_.sweep.dbm_offset_db)
        {
            d.noise_floor_dbm = s.noise_db + *cfg_.sweep.dbm_offset_db;
            d.power_dbm       = s.peak_db + *cfg_.sweep.dbm_offset_db;
        }
        d.iq_snapshot              = dwell_snapshot;  // shared_ptr, zero-copy
        d.snapshot_sample_rate_sps = sample_rate;
        out.push_back(d);
    }

    // Decay signals not seen this dwell; expire stale entries entirely.
    for (auto it2 = pmap.begin(); it2 != pmap.end();)
    {
        if (!seen_this_dwell.count(it2->first))
        {
            if (++it2->second.misses > PERSIST_MAX_MISSES)
                it2 = pmap.erase(it2);
            else
                ++it2;
        }
        else
        {
            ++it2;
        }
    }

    return out;
}

void SpectrumScanner::accumulateNoiseMap(int                     ch,
                                         uint64_t                key,
                                         double                  center_hz,
                                         double                  sample_rate_hz,
                                         const FftProcessor&     proc,
                                         const NoiseFloorResult& nf)
{
    const auto [lo, hi] = proc.usableRange(static_cast<float>(cfg_.sweep.usable_bw_fraction));
    const int   n       = hi - lo + 1;
    const auto& pdb     = proc.powerDb();
    auto&       a       = ch_map_[(size_t)ch][key];
    if (a.lo != lo || a.hi != hi || a.fft != proc.fft_size())
    {
        a     = NoiseAccum{};
        a.lo  = lo;
        a.hi  = hi;
        a.fft = proc.fft_size();
        a.floor_sum.assign((size_t)n, 0.f);
        a.quiet.assign((size_t)n, 1e30f);
        a.occ.assign((size_t)n, 0.f);
    }
    for (int i = 0; i < n; ++i)
    {
        const int   k = lo + i;
        const float p = pdb[(size_t)k];
        a.floor_sum[(size_t)i] += nf.floor_db[(size_t)k];
        a.quiet[(size_t)i] = std::min(a.quiet[(size_t)i], p);
        if (p >= nf.floor_db[(size_t)k] + nf.margin_db)
            a.occ[(size_t)i] += 1.f;
    }
    if (++a.sweeps < cfg_.noise_map.every_sweeps)
        return;

    // Reduce to cfg_.noise_map.bins points (0 = every examined bin).
    const int    g      = cfg_.noise_map.bins > 0
                              ? std::max(1, (n + cfg_.noise_map.bins - 1) / cfg_.noise_map.bins)
                              : 1;
    const double bin_hz = sample_rate_hz / proc.fft_size();
    NoiseMap     m;
    m.timestamp      = system_clock::now();
    m.scanner_id     = cfg_.scanner_id;
    m.channel        = ch;
    m.center_freq_hz = center_hz;
    m.sample_rate_hz = sample_rate_hz;
    m.f_start_hz     = center_hz + ((double)lo - proc.fft_size() / 2 - 0.5) * bin_hz;
    m.point_hz       = g * bin_hz;
    m.sweeps         = a.sweeps;
    m.dbm_offset_db  = cfg_.sweep.dbm_offset_db;
    for (int s = 0; s < n; s += g)
    {
        const int e     = std::min(n, s + g);
        double    fsum  = 0.0;
        float     quiet = -1e30f, occ = 0.f;
        for (int i = s; i < e; ++i)
        {
            fsum += a.floor_sum[(size_t)i] / (float)a.sweeps;
            quiet = std::max(quiet, a.quiet[(size_t)i]);
            occ   = std::max(occ, a.occ[(size_t)i] / (float)a.sweeps);
        }
        m.floor_db.push_back((float)(fsum / (e - s)));
        m.quiet_db.push_back(quiet);
        m.occupancy.push_back(occ);
    }
    a.sweeps = 0;
    std::fill(a.floor_sum.begin(), a.floor_sum.end(), 0.f);
    std::fill(a.quiet.begin(), a.quiet.end(), 1e30f);
    std::fill(a.occ.begin(), a.occ.end(), 0.f);
    map_cb_(m);
}

}  // namespace acq

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
