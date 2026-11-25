#pragma once
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
#include <atomic>

namespace acq
{

/// Set by the SIGINT / SIGTERM handler (a lock-free atomic store is
/// async-signal-safe). Long start-up waits poll it: Acq used to ignore
/// SIGTERM for minutes while the broker or controller was unreachable
/// (a 60 s channel wait, then up to 300 s of HEALTH_QUERY retries), so
/// `systemctl stop` / `podman stop` ended in SIGKILL (ACQ-29).
inline std::atomic<bool>& shutdownRequested()
{
    static std::atomic<bool> flag{false};
    return flag;
}

}  // namespace acq

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
