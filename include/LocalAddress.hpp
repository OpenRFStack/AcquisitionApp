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
 * @file LocalAddress.hpp
 * @brief Local IPv4 address of the route towards a URL's host.
 *
 * A receiver bound to 0.0.0.0 still has to tell SdrResourceManager where to
 * send its UDP stream. 127.0.0.1 is right only when the controller is on the
 * same host; the address the kernel would use to reach the broker is the best
 * available guess for a remote controller (the controller and the broker are
 * normally co-located). No packet is sent: connect() on a UDP socket only
 * selects the route.
 */
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <string>

namespace acq {

/// "amqp://user:pw@host:5672/vhost" → "host"; "[::1]:5672" → "::1".
inline std::string urlHost(const std::string& url) {
    std::string s = url;
    if (auto p = s.find("://"); p != std::string::npos) s = s.substr(p + 3);
    if (auto p = s.find('/'); p != std::string::npos) s = s.substr(0, p);
    if (auto p = s.rfind('@'); p != std::string::npos) s = s.substr(p + 1);
    if (!s.empty() && s[0] == '[') {
        auto e = s.find(']');
        return e == std::string::npos ? s.substr(1) : s.substr(1, e - 1);
    }
    if (auto p = s.rfind(':'); p != std::string::npos) s = s.substr(0, p);
    return s;
}

/// Local IPv4 address used to reach the host of @p url; @p fallback on failure.
inline std::string localAddressTowards(const std::string& url,
                                       const std::string& fallback = "127.0.0.1") {
    const std::string host = urlHost(url);
    if (host.empty()) return fallback;
    addrinfo hints{};
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo* res = nullptr;
    if (::getaddrinfo(host.c_str(), "9", &hints, &res) != 0 || !res) return fallback;
    std::string out = fallback;
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd >= 0) {
        if (::connect(fd, res->ai_addr, res->ai_addrlen) == 0) {
            sockaddr_in sa{};
            socklen_t sl = sizeof sa;
            char buf[INET_ADDRSTRLEN] = {};
            if (::getsockname(fd, reinterpret_cast<sockaddr*>(&sa), &sl) == 0 &&
                ::inet_ntop(AF_INET, &sa.sin_addr, buf, sizeof buf) && sa.sin_addr.s_addr != 0)
                out = buf;
        }
        ::close(fd);
    }
    ::freeaddrinfo(res);
    return out;
}

} // namespace acq

/*
========================================================================
End of file — OpenRFStack
Subject to Personal Use License
https://github.com/OpenRFStack
========================================================================
*/
