/*
This file is part of BCP-MAPF.

BCP-MAPF is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

BCP-MAPF is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with BCP-MAPF.  If not, see <https://www.gnu.org/licenses/>.

Author: Edward Lam <ed@ed-lam.com>
*/

#ifndef TRUFFLEHOG_COORDINATES_H
#define TRUFFLEHOG_COORDINATES_H

#include "Includes.h"

namespace TruffleHog
{

using Location = Int;
using Time = Int;
enum Direction: uint8_t
{
    NORTH = 0,
    SOUTH = 1,
    EAST = 2,
    WEST = 3,
    WAIT = 4,
    INVALID = 5
};

union Edge
{
    struct
    {
        Location n: 29;
        Direction d: 3;
    };
    uint32_t id;

    Edge() noexcept = default;
    explicit Edge(const Location n, const Direction d) noexcept : n(n), d(d) {}
};
static_assert(sizeof(Edge) == 4);
static_assert(std::is_trivial<Edge>::value);
inline bool operator==(const Edge a, const Edge b)
{
    return a.n == b.n && a.d == b.d;
}
inline bool operator!=(const Edge a, const Edge b)
{
    return !(a == b);
}

union LocationTime
{
    struct
    {
        Location n;
        Time t;
    };
    uint64_t nt;

    LocationTime() noexcept = default;
    LocationTime(const uint64_t nt) noexcept : nt(nt) {}
    explicit LocationTime(const Location n, const Time t) noexcept : n(n), t(t) {}
};
static_assert(sizeof(LocationTime) == 8);
static_assert(std::is_trivial<LocationTime>::value);
inline bool operator==(const LocationTime a, const LocationTime b)
{
    return a.nt == b.nt;
}
inline bool operator!=(const LocationTime a, const LocationTime b)
{
    return a.nt != b.nt;
}

union EdgeTime
{
    struct
    {
        Edge e;
        Time t;
    } et;
    struct
    {
        Location n : 29;
        Direction d : 3;
        Time t;
    };
    uint64_t id;

    EdgeTime() noexcept = default;
    explicit EdgeTime(const Edge e, const Time t) noexcept : et{e, t} {}
    explicit EdgeTime(const LocationTime nt, const Direction d) noexcept : n{nt.n}, d{d}, t{nt.t} {}
    explicit EdgeTime(const Location n, const Direction d, const Time t) noexcept : n{n}, d{d}, t{t} {}

    inline LocationTime nt() const noexcept { return LocationTime{n, t}; }
};
static_assert(sizeof(EdgeTime) == 8);
static_assert(std::is_trivial<EdgeTime>::value);
inline bool operator==(const EdgeTime a, const EdgeTime b)
{
    return a.id == b.id;
}
inline bool operator!=(const EdgeTime a, const EdgeTime b)
{
    return !(a == b);
}

}

namespace robin_hood
{

template<>
struct hash<TruffleHog::Edge>
{
    inline std::size_t operator()(const TruffleHog::Edge e) const noexcept
    {
        return robin_hood::hash<uint32_t>{}(e.id);
    }
};

template<>
struct hash<TruffleHog::LocationTime>
{
    inline std::size_t operator()(const TruffleHog::LocationTime nt) const noexcept
    {
        return robin_hood::hash<uint64_t>{}(nt.nt);
    }
};

template<>
struct hash<TruffleHog::EdgeTime>
{
    inline std::size_t operator()(const TruffleHog::EdgeTime et) const noexcept
    {
        return robin_hood::hash<uint64_t>{}(et.id);
    }
};

}

namespace fmt
{

template<>
struct formatter<TruffleHog::Direction>: formatter<string_view>
{
    template<typename FormatContext>
    inline auto format(const TruffleHog::Direction d, FormatContext& ctx)
    {
        string_view name = "INVALID";
        switch (d)
        {
            case TruffleHog::Direction::NORTH: name = "NORTH"; break;
            case TruffleHog::Direction::SOUTH: name = "SOUTH"; break;
            case TruffleHog::Direction::EAST:  name = "EAST";  break;
            case TruffleHog::Direction::WEST:  name = "WEST";  break;
            case TruffleHog::Direction::WAIT:  name = "WAIT";  break;
            default: break;
        }
        return formatter<string_view>::format(name, ctx);
    }
};

template<>
struct formatter<TruffleHog::Edge>
{
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx) { return ctx.begin(); }

    template<typename FormatContext>
    inline auto format(const TruffleHog::Edge e, FormatContext& ctx)
    {
        return format_to(ctx.out(), "(n={},d={})", e.n, e.d);
    }
};

template<>
struct formatter<TruffleHog::LocationTime>
{
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx) { return ctx.begin(); }

    template<typename FormatContext>
    inline auto format(const TruffleHog::LocationTime nt, FormatContext& ctx)
    {
        return format_to(ctx.out(), "(n={},t={})", nt.n, nt.t);
    }
};

template<>
struct formatter<TruffleHog::EdgeTime>
{
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx) { return ctx.begin(); }

    template<typename FormatContext>
    inline auto format(const TruffleHog::EdgeTime et, FormatContext& ctx)
    {
        return format_to(ctx.out(), "(n={},d={},t={})", et.n, et.d, et.t);
    }
};

}

//template<class T>
//inline void hash_combine(std::size_t& s, const T& v)
//{
//    s ^= robin_hood::hash<T>(v) + 0x9e3779b9 + (s << 6) + (s >> 2);
//}

#endif
