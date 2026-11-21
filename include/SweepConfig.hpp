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
#include <cmath>
#include <vector>
/**
 * @file SweepConfig.hpp
 * @brief Configuration structs for AcquisitionApp, loaded from XML.
 *
 * SweepConfig::from_file() parses the scanner.xml config file.
 * All sub-structs (AmqpConfig, DeviceConfig, SweepParams, etc.) are
 * populated from the corresponding XML elements.
 */
#include <algorithm>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

#include <au/prefix.hh>
#include <au/units/hertz.hh>
#include <au/units/seconds.hh>

namespace acq
{

/// @brief AMQP broker connection and topic configuration.
struct AmqpConfig
{
    std::string url{"amqp://localhost:5672"};              ///< Broker URL.
    std::string username;                                  ///< AMQP username.
    std::string password;                                  ///< AMQP password.
    std::string detection_topic{"rf.detections"};          ///< Topic for RF_DETECTION messages.
    std::string task_request_queue{"sdr.task.request"};    ///< Queue for outbound task requests.
    std::string task_response_queue{"sdr.task.response"};  ///< Queue for inbound task responses.
    int         reconnect_interval_sec{5};  ///< Seconds between reconnect attempts on AMQP error.
};

/// @brief PostgreSQL detection database connection parameters.
struct DbConfig
{
    std::string host{"localhost"};    ///< Database host.
    int         port{5432};           ///< Database port.
    std::string name{"sdr_scanner"};  ///< Database name.
    std::string user;                 ///< Database user.
    std::string password;             ///< Database password.
    /// @brief Build a libpqxx connection string from these parameters.
    std::string connection_string() const;
};

/// @brief SDR hardware configuration.
struct DeviceConfig
{
    int                      rx_channels{1};                ///< Number of RX channels to request.
    au::QuantityD<au::Hertz> sample_rate{au::hertz(10e6)};  ///< Sample rate (samples/s).
    double                   rx_gain_db{40.0};              ///< RX gain (dB).
    /// Hardware AGC (e.g. the AD9361's). Protects against overload from strong
    /// signals, but the gain then differs per entry, so Detection::power_db
    /// is relative to the AGC state rather than absolute. Detection itself
    /// (CFAR, relative to the local floor) is unaffected. Default off.
    bool                     rx_agc{false};
    au::QuantityD<au::Hertz> bandwidth_hz{
        au::hertz(10e6)};  ///< Requested RF bandwidth per channel.
    /// When non-empty: hard-pin all tasks from this scanner to this device and
    /// suppress auto-split (both slices would need the same device anyway on
    /// shared-LO hardware, so splitting just causes sequential retries).
    std::string required_device_id;
};

/**
 * @brief Frequency sweep and signal detection parameters.
 *
 * These parameters control the core detection pipeline.
 * See the README "Sweep parameters" table for the effect of each value.
 */
struct SweepParams
{
    au::QuantityD<au::Hertz> start_hz{au::hertz(70'000'000.0)};    ///< Sweep start frequency.
    au::QuantityD<au::Hertz> stop_hz{au::hertz(1'000'000'000.0)};  ///< Sweep stop frequency.
    /// IQ samples per dwell (≥ fft_size). 0 = auto: every sample of the
    /// controller's minimum scan dwell (5 ms) — streamed anyway; more Welch
    /// frames make the floor estimate tighter (see effectiveDwellSamples()).
    int    dwell_samples{0};
    int    fft_size{4096};            ///< FFT size (power of 2, ≥ 64).
    double usable_bw_fraction{0.80};  ///< Fraction of FFT bins examined (discards roll-off edges).
    double threshold_db{10.0};        ///< Detection threshold above local CA-CFAR noise (dB).
    au::QuantityD<au::Hertz> min_signal_bw_hz{
        au::hertz(1'000.0)};  ///< Minimum run width to report.
    int settle_samples{512};  ///< Samples discarded after each retune.
    /// Bins within this range of DC are blanked to suppress AD9361 LO leakage.
    /// Set 0 to disable.
    au::QuantityD<au::Hertz> dc_guard_hz{au::hertz(50'000.0)};
    int                      cfar_guard_bins{8};  ///< CA-CFAR guard cells each side of test bin.
    int cfar_ref_bins{32};  ///< CA-CFAR reference cells each side for noise average.
    /// Minimum peak-to-mean power ratio (dB) for multi-bin runs.
    /// Flat noise bumps below this threshold are rejected.
    float min_papr_db{3.0f};
    /// @name Noise floor (<noise_floor> in <sweep>) — NoiseFloorEstimator.hpp
    /// @{
    /// "dsp" (default): ordered-statistic + minimum-statistics floor and a
    /// false-alarm-rate margin; "cfar": censored CA-CFAR with threshold_db;
    /// "ml": reserved (falls back to "dsp").
    std::string noise_method{"dsp"};
    double      noise_pfa{3e-3};         ///< per-bin false-alarm probability
    double      noise_percentile{0.25};  ///< per-block ordered statistic
    int         noise_block_bins{64};    ///< bins per block
    int         noise_history{8};        ///< sweeps of minimum statistics (1 = off)
    float       noise_max_tilt_db{6.f};  ///< block floor clamp around the band
    /// dBFS → dBm calibration at the configured gain (front-end specific).
    /// When set, detections also carry the floor and peak in dBm.
    std::optional<double> dbm_offset_db;
    /// @}
};

/// @brief Per-bin noise map publication (<noise_map>, off by default).
///
/// Every `every_sweeps` sweeps of a scan entry, one RF_NOISE_MAP message with
/// floor_db (the estimator's floor), quiet_db (each bin's own minimum level
/// over the window — a continuous carrier shows here, it is never fed back
/// into detection) and occupancy (fraction of sweeps above the detection
/// margin), reduced to `bins` points (0 = every examined FFT bin).
struct NoiseMapConfig
{
    bool        enabled{false};
    int         every_sweeps{10};
    int         bins{256};
    std::string topic{"rf.noise_map"};
};

/// @brief UDP receive parameters for IQ data from SdrResourceManager.
struct ReceiverConfig
{
    std::string local_ip{"0.0.0.0"};  ///< Local IP to bind (must be routable from the controller).
    int         port{0};              ///< UDP port to pre-bind.  0 = OS-assigned.
};

/**
 * @brief Threat detection configuration (GPS jamming/spoofing, content-layer validators).
 *
 * Controls which threat monitors are active. All disabled by default so the
 * system behaves identically to a version without threat detection unless
 * explicitly enabled in scanner.xml.
 */
struct ThreatConfig
{
    bool enabled{false};  ///< Master switch — disables all threat detection when false.

    // ── GPS frequency monitoring ─────────────────────────────────────────────
    bool   gps_enabled{false};             ///< Scan L1/L2/L5 for jammers and spoofers.
    double gps_jammer_threshold_db{20.0};  ///< dB above EMA baseline to declare jamming.
    double gps_spoof_detectable_dbm{
        -110.0};                        ///< Any GPS-band signal above this = spoofing suspect.
    double gps_check_interval_s{30.0};  ///< Seconds between L1/L2/L5 checks.

    // ── Content-layer validators (run inside DemodApp demodulators) ──────────
    bool adsb_enabled{true};  ///< ADS-B physics validation (impossible altitude/speed/CRC rate).
    bool ais_enabled{true};   ///< AIS impossible vessel speed / invalid MMSI.
    bool eas_enabled{true};   ///< EAS/SAME invalid originator or rare event code.
    bool dsc_enabled{true};   ///< DSC distress call MMSI anomaly detection.
    bool p25_rogue_enabled{true};  ///< P25 rogue site (unexpected WACN change).

    // ── Alert persistence ────────────────────────────────────────────────────
    std::string alert_topic{"rf.alerts"};  ///< AMQP topic alerts are published to by DemodApp.
};

/**
 * @brief P25 trunked-system voice channel follower configuration.
 *
 * When enabled, AcquisitionApp subscribes to @p grant_topic and tunes a
 * dedicated IqSource to each voice channel grant published by DemodApp,
 * capturing IQ for @p capture_s seconds per grant.
 */
struct P25Config
{
    bool        enabled{false};                ///< Enable voice channel following.
    std::string grant_topic{"rf.p25.grants"};  ///< AMQP topic DemodApp publishes grants to.
    double      capture_s{3.0};                ///< IQ capture duration per voice grant (s).
    int         rank{3};                       ///< Preemption rank for voice captures (highest).
    std::vector<uint32_t> tg_whitelist;        ///< Talk groups to follow; empty = all.
};

/**
 * @brief One frequency band for multi-band parallel scanning.
 *
 * Each BandConfig spawns its own IqSource + SpectrumScanner, requesting
 * whichever SDR device is next available from SdrResourceManager (or a
 * specific device if device_id is set). All sweep parameters (FFT size,
 * CFAR, threshold, etc.) are inherited from the top-level SweepParams and
 * only start_hz / stop_hz are overridden per band.
 *
 * XML syntax inside <bands>:
 * @code{.xml}
 *   <band>
 *     <start_hz>88000000</start_hz>
 *     <stop_hz>174000000</stop_hz>
 *     <!-- <device>pluto-0</device>  optional: pin to a specific device -->
 *   </band>
 * @endcode
 */
struct BandConfig
{
    std::string device_id;               ///< Empty = SdrRM assigns next available.
    bool        device_required{false};  ///< When true, device_id is a hard constraint.
    au::QuantityD<au::Hertz> start_hz{au::hertz(0.0)};  ///< Band start frequency.
    au::QuantityD<au::Hertz> stop_hz{au::hertz(0.0)};   ///< Band stop frequency.
};

/// SdrResourceManager rejects scan entries shorter than this (doAcceptScan:
/// dwell_ms < 5 → SCAN_ENTRY_INVALID).
inline constexpr int kControllerMinDwellMs = 5;

/// dwell_ms for one scan entry: long enough to deliver @p dwell_samples at
/// @p sample_rate_sps, never below the controller's minimum. Truncating to
/// whole milliseconds used to give 1 ms for the shipped scanner.xml (4096
/// samples at 10 MS/s = 0.41 ms) and every scan request was rejected.
inline int scanDwellMs(int dwell_samples, double sample_rate_sps)
{
    if (sample_rate_sps <= 0.0)
        return kControllerMinDwellMs;
    const double ms = std::ceil(dwell_samples * 1000.0 / sample_rate_sps);
    return ms < kControllerMinDwellMs ? kControllerMinDwellMs : (int)ms;
}

/// Samples per dwell: dwell_samples, or for 0 (auto) 80 % of what the
/// controller's minimum scan dwell streams (the timer can deliver a little
/// less than fs × 5 ms; a target above that would never complete the last
/// entry of a sweep), in whole Welch hops, at least fft_size.
inline int effectiveDwellSamples(const SweepParams& sw, double sample_rate_sps)
{
    if (sw.dwell_samples > 0)
        return sw.dwell_samples;
    const double avail = 0.8 * sample_rate_sps * kControllerMinDwellMs / 1000.0;
    const int    hop   = sw.fft_size / 2;
    const int    n     = (int)((avail - sw.fft_size) / hop) * hop + sw.fft_size;
    return std::clamp(n, sw.fft_size, 1 << 20);
}

/**
 * @brief Centre frequencies of one SCAN task's entries (ACQ-06).
 *
 * Entries sit on a fixed grid: centre_k = start + (k + ½)·step for every k
 * with centre_k ≤ stop. The list starts at the grid entry containing
 * @p resume_hz and wraps, so a re-submission after preemption continues the
 * sweep and every grid frequency appears exactly once. The grid never moves
 * — the per-frequency noise-floor and persistence state is keyed by these
 * centres. A span narrower than one step (e.g. a 12.5 kHz P25 voice channel)
 * yields one entry at the span centre; it used to yield none, and the
 * controller rejects an empty scan.
 */
inline std::vector<double>
scanEntryCenters(double start_hz, double stop_hz, double step_hz, double resume_hz)
{
    std::vector<double> grid;
    if (step_hz > 0.0)
        for (int k = 0; start_hz + (k + 0.5) * step_hz <= stop_hz; ++k)
            grid.push_back(start_hz + (k + 0.5) * step_hz);
    if (grid.empty())
        return {0.5 * (start_hz + stop_hz)};
    int first = 0;
    if (resume_hz > start_hz)
    {
        first = (int)std::floor((resume_hz - start_hz) / step_hz + 1e-9);
        if (first >= (int)grid.size())
            first = 0;
    }
    std::vector<double> out;
    out.reserve(grid.size());
    for (size_t i = 0; i < grid.size(); ++i)
        out.push_back(grid[(first + i) % grid.size()]);
    return out;
}

/**
 * @brief Where the next SCAN task resumes after the dwell at @p completed_hz
 *        finished: the start of the following grid entry (wrapping to start).
 *
 * The old rule, `packet centre + step` on every packet, landed half a step
 * off the grid, so each preemption shifted the whole grid (unbounded growth of
 * the per-frequency maps, lost persistence history) and skipped the entry
 * that was cut short.
 */
inline double resumeAfterDwell(double start_hz, double stop_hz, double step_hz, double completed_hz)
{
    if (step_hz <= 0.0)
        return start_hz;
    const int    k    = (int)std::floor((completed_hz - start_hz) / step_hz);
    const double next = start_hz + (k + 1) * step_hz;
    return (next + 0.5 * step_hz > stop_hz) ? start_hz : next;
}

struct SweepConfig
{
    std::string scanner_id{"scanner-0"};  ///< Identifies this scanner in AMQP messages.
    int         rank{1};  ///< Preemption tier: 1=Acq (lowest), 2=Ana, 3=DF (highest).
    /// Duration to sleep after each sweep pass before re-submitting the SCAN task.
    /// Gives AnalysisApp (lower rank) a guaranteed window to grab the SDR.
    /// 0 = disabled (continuous sweep, no analysis window).
    au::QuantityD<au::Seconds> analysis_pause_ms{au::seconds(0.0)};

    /**
     * @brief Multi-band parallel scan configuration.
     *
     * When non-empty, one SpectrumScanner is spawned per band and scan_device_ids
     * is ignored. Each scanner independently requests "next available device" from
     * SdrResourceManager, so N bands naturally spread across N available SDRs with
     * no duplicate frequency coverage and no inter-process coordination required.
     *
     * When empty, falls back to the legacy scan_device_ids / single-sweep behaviour.
     */
    std::vector<BandConfig> bands;

    /**
     * @brief Legacy fixed-device scan list.
     *
     * When non-empty and @p bands is empty, one scanner is spawned per device
     * ID, all scanning the same sweep range. Superseded by @p bands + auto-split;
     * kept for backwards compatibility with existing deployments.
     */
    std::vector<std::string> scan_device_ids;

    AmqpConfig     amqp;       ///< AMQP broker connection and topic config.
    DbConfig       db;         ///< PostgreSQL connection config.
    DeviceConfig   device;     ///< SDR hardware capabilities to request from SdrRM.
    SweepParams    sweep;      ///< Frequency sweep and detection parameters (shared defaults).
    ReceiverConfig receiver;   ///< UDP IQ receive socket config.
    P25Config      p25;        ///< P25 voice channel follower (disabled by default).
    ThreatConfig   threat;     ///< Threat detection monitors (all disabled by default).
    NoiseMapConfig noise_map;  ///< Per-bin noise map publication (disabled by default).

    /**
     * @brief Load SweepConfig from an XML file.
     * @param path Path to the scanner.xml configuration file.
     * @return Populated SweepConfig.
     * @throws std::runtime_error if the file cannot be opened or parsed.
     */
    static SweepConfig from_file(const std::string& path);

    /**
     * @brief Distribute @p n_devices across @p bands by subdividing each band
     *        into equal frequency slices.
     *
     * Called at startup after queryDeviceCount() to ensure every available
     * SDR covers a unique non-overlapping frequency range.
     *
     * If n_devices ≤ bands.size() the input is returned unchanged (no split).
     * Otherwise each band is divided into (n_devices / bands.size()) slices,
     * with the remainder distributed one extra slice to the first bands:
     *
     * @code
     *   splitBands({[70–6000]},     4) → 4 equal slices of [70–6000]
     *   splitBands({[70–1100],[1100–6000]}, 4) → each band → 2 slices (4 total)
     *   splitBands({A,B,C},         4) → A→2 slices, B→1, C→1 (4 total)
     *   splitBands({A,B,C,D},       4) → 1:1, no split
     *   splitBands({A,B,C,D,E},     4) → unchanged (5 > 4)
     * @endcode
     *
     * @param bands     Input bands (must be non-empty).
     * @param n_devices Number of online devices reported by SdrRM.
     * @return Expanded band list with device_id inherited from the parent band.
     */
    static std::vector<BandConfig> splitBands(const std::vector<BandConfig>& bands, int n_devices);
};

/// @brief Config for the P25 voice-capture IQ source: the sweep config with
/// the voice-capture rank. The source used to be built from the sweep config
/// as is, so voice captures went out at the sweep's rank (1) and could never
/// preempt the sweep holding the radio; <p25><rank> was parsed, never used.
inline SweepConfig p25SourceConfig(const SweepConfig& cfg)
{
    SweepConfig out = cfg;
    out.rank        = cfg.p25.rank;
    return out;
}

}  // namespace acq

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
