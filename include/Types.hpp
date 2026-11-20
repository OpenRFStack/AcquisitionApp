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
 * @file Types.hpp
 * @brief Detection result type for AcquisitionApp.
 *
 * A Detection is produced by SpectrumScanner for each confirmed signal and
 * forwarded to AmqpPublisher (AMQP) and DetectionDb (PostgreSQL).
 */
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <au/units/hertz.hh>
#include <au/units/seconds.hh>

/// @namespace acq
/// @brief AcquisitionApp internals.
namespace acq
{

/// @brief RF_DETECTION schema version emitted by this build.
inline constexpr const char* SCHEMA_VERSION =
    "1.3";  // 1.2: base64 iq_snapshot; 1.3: snr_db = SNR, papr_db, noise_floor_db[m]

/**
 * @brief A confirmed RF signal detection from one dwell of a frequency sweep.
 *
 * Detections are produced by SpectrumScanner::processDwell() after the
 * persistence filter confirms the signal in at least two consecutive sweeps
 * (unless PAPR ≥ PERSIST_BYPASS_PAPR_DB, in which case it emits immediately).
 *
 * The @p iq_snapshot is shared across all detections from the same dwell
 * via shared_ptr — zero copies regardless of how many signals are found.
 */
struct Detection
{
    std::chrono::system_clock::time_point timestamp;  ///< Wall-clock time of detection.
    au::QuantityD<au::Hertz>              center_freq_hz{
        au::hertz(0.0)};  ///< Sub-bin interpolated centre frequency.
    au::QuantityD<au::Hertz> bandwidth_hz{au::hertz(0.0)};  ///< Estimated occupied bandwidth.
    float                    power_db{0.f};  ///< Peak power in the detection run (dBFS).
    std::string              scanner_id;     ///< Scanner identifier from SweepConfig.
    int                      channel{0};     ///< RX channel index (0-based).
    /// Peak above the local noise floor (dB). It used to carry the PAPR.
    float snr_db{0.f};
    float papr_db{0.f};         ///< Peak − mean power over the detection run (dB).
    float noise_floor_db{0.f};  ///< Noise floor under the detection (dBFS per bin).
    /// Floor and peak in dBm when <noise_floor><dbm_offset_db> is configured.
    std::optional<double> noise_floor_dbm;
    std::optional<double> power_dbm;

    /// 1 024-sample IQ snapshot from the detecting dwell (interleaved float32 I,Q,…).
    /// Shared across all detections from the same dwell — zero-copy shared_ptr.
    /// Null when no snapshot was collected (unit tests or snapshot disabled).
    std::shared_ptr<const std::vector<float>> iq_snapshot;
    au::QuantityD<au::Hertz>                  snapshot_sample_rate_sps{
        au::hertz(0.0)};  ///< Sample rate of the IQ snapshot.

    /// @name GPS position (from GpsApp — nullopt when GpsApp is not running or has no fix)
    /// @{
    std::optional<double> lat;    ///< Latitude (decimal degrees, WGS-84) at time of detection.
    std::optional<double> lon;    ///< Longitude (decimal degrees, WGS-84) at time of detection.
    std::optional<double> alt_m;  ///< Altitude above MSL (metres) at time of detection.
    /// @}
};

/**
 * @brief Per-bin noise / occupancy map of one scan entry (RF_NOISE_MAP).
 *
 * Point i covers [f_start_hz + i·point_hz, f_start_hz + (i+1)·point_hz).
 * When several FFT bins share a point: floor_db is their mean, quiet_db their
 * maximum (a narrow persistent carrier must stay visible) and occupancy the
 * maximum.
 */
struct NoiseMap
{
    std::chrono::system_clock::time_point timestamp;
    std::string                           scanner_id;
    int                                   channel{0};
    double                                center_freq_hz{0.0};  ///< scan entry centre
    double                                sample_rate_hz{0.0};
    double                                f_start_hz{0.0};  ///< low edge of point 0
    double                                point_hz{0.0};    ///< width of one point
    int                                   sweeps{0};        ///< sweeps in this window
    std::vector<float>                    floor_db;  ///< estimated noise floor (dBFS per FFT bin)
    std::vector<float>    quiet_db;       ///< per-bin minimum level over the window (dBFS)
    std::vector<float>    occupancy;      ///< fraction of sweeps above floor + margin
    std::optional<double> dbm_offset_db;  ///< dBFS → dBm, when configured
};

}  // namespace acq

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
