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
#include "P25Db.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <memory>

#include <pqxx/pqxx>

namespace acq
{

// Connects on first use and after a lost connection (at most every 5 s).
// It used to connect in the constructor: with the database down at start-up
// main caught the exception and P25 channels were never recorded that run;
// after a failed reconnect it kept using the broken connection.
struct P25Db::Impl
{
    std::string                           conn_str;
    std::unique_ptr<pqxx::connection>     conn;
    std::chrono::steady_clock::time_point next_try{};
    bool                                  warned = false;

    explicit Impl(const std::string& cs)
        : conn_str(cs)
    {
    }

    pqxx::connection* get()
    {
        if (conn)
            return conn.get();
        const auto now = std::chrono::steady_clock::now();
        if (now < next_try)
            return nullptr;
        try
        {
            conn = std::make_unique<pqxx::connection>(conn_str);
            if (warned)
                spdlog::info("[P25Db] connected");
            warned = false;
            return conn.get();
        }
        catch (const std::exception& e)
        {
            if (!warned)
                spdlog::warn("[P25Db] database unavailable: {} — retrying", e.what());
            warned   = true;
            next_try = now + std::chrono::seconds(5);
            return nullptr;
        }
    }

    void lost()
    {
        conn.reset();
    }
};

P25Db::P25Db(const std::string& conn_str)
    : impl_(std::make_unique<Impl>(conn_str))
{
}

P25Db::~P25Db() = default;

void P25Db::upsert_control(double freq_hz,
                           int    wacn,
                           int    sys_id,
                           int    rfss_id,
                           int    site_id,
                           int    channel_iden,
                           int    channel_num)
{
    try
    {
        auto* c = impl_->get();
        if (!c)
            return;
        pqxx::work tx(*c);
        tx.exec_params(R"sql(
            INSERT INTO p25_channels
                (freq_hz, wacn, sys_id, rfss_id, site_id,
                 channel_iden, channel_num, is_control, grant_count)
            VALUES ($1,$2,$3,$4,$5,$6,$7,true,1)
            ON CONFLICT (freq_hz) WHERE is_control = true DO UPDATE SET
                last_seen    = now(),
                wacn         = EXCLUDED.wacn,
                sys_id       = EXCLUDED.sys_id,
                rfss_id      = EXCLUDED.rfss_id,
                site_id      = EXCLUDED.site_id,
                grant_count  = p25_channels.grant_count + 1
        )sql",
                       freq_hz,
                       wacn,
                       sys_id,
                       rfss_id,
                       site_id,
                       channel_iden >= 0 ? std::optional<int>(channel_iden) : std::nullopt,
                       channel_num >= 0 ? std::optional<int>(channel_num) : std::nullopt);
        tx.commit();
        spdlog::debug("[P25Db] control channel {:.4f} MHz recorded", freq_hz / 1e6);
    }
    catch (const pqxx::broken_connection& e)
    {
        spdlog::error("[P25Db] connection lost: {} — reconnecting", e.what());
        impl_->lost();
    }
    catch (const std::exception& e)
    {
        spdlog::warn("[P25Db] upsert_control failed: {}", e.what());
    }
}

void P25Db::upsert_grant(const P25Grant& g,
                         int             wacn,
                         int             sys_id,
                         int             rfss_id,
                         int             site_id,
                         int             channel_iden,
                         int             channel_num)
{
    try
    {
        auto* c = impl_->get();
        if (!c)
            return;
        pqxx::work tx(*c);
        // alg_id / alg_name / key_id may be 0/unknown if HDU not yet decoded
        std::optional<int>         alg_id_opt;
        std::optional<std::string> alg_name_opt;
        std::optional<int>         key_id_opt;
        if (g.encrypted)
        {
            // Use HDU info if available; otherwise mark as "Encrypted (unknown alg)"
            alg_id_opt   = static_cast<int>(g.alg_id);
            alg_name_opt = g.alg_name;
            if (g.key_id)
                key_id_opt = static_cast<int>(g.key_id);
        }

        tx.exec_params(R"sql(
            INSERT INTO p25_channels
                (freq_hz, wacn, sys_id, rfss_id, site_id,
                 channel_iden, channel_num, is_control,
                 talk_group, source_id, encrypted, emergency,
                 alg_id, alg_name, key_id, grant_count)
            VALUES ($1,$2,$3,$4,$5,$6,$7,false,$8,$9,$10,$11,$12,$13,$14,1)
            ON CONFLICT (freq_hz, talk_group) DO UPDATE SET
                last_seen   = now(),
                source_id   = EXCLUDED.source_id,
                encrypted   = EXCLUDED.encrypted,
                emergency   = EXCLUDED.emergency,
                alg_id      = COALESCE(EXCLUDED.alg_id,   p25_channels.alg_id),
                alg_name    = COALESCE(EXCLUDED.alg_name, p25_channels.alg_name),
                key_id      = COALESCE(EXCLUDED.key_id,   p25_channels.key_id),
                grant_count = p25_channels.grant_count + 1
        )sql",
                       g.freq_hz,
                       wacn,
                       sys_id,
                       rfss_id,
                       site_id,
                       channel_iden >= 0 ? std::optional<int>(channel_iden) : std::nullopt,
                       channel_num >= 0 ? std::optional<int>(channel_num) : std::nullopt,
                       (int)g.talk_group,
                       (int)g.source_id,
                       g.encrypted,
                       g.emergency,
                       alg_id_opt,
                       alg_name_opt,
                       key_id_opt);
        tx.commit();
        spdlog::debug("[P25Db] grant TG={} {:.4f}MHz recorded", g.talk_group, g.freq_hz / 1e6);
    }
    catch (const pqxx::broken_connection& e)
    {
        spdlog::error("[P25Db] connection lost: {} — reconnecting", e.what());
        impl_->lost();
    }
    catch (const std::exception& e)
    {
        spdlog::warn("[P25Db] upsert_grant failed: {}", e.what());
    }
}

}  // namespace acq

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
