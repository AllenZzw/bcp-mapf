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
using Timepoint = Int;
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

union LocationTimepoint
{
    struct
    {
        Location n;
        Timepoint t;
    };
    uint64_t nt;

    LocationTimepoint() noexcept = default;
    LocationTimepoint(const uint64_t nt) noexcept : nt(nt) {}
    explicit LocationTimepoint(const Location n, const Timepoint t) noexcept : n(n), t(t) {}
};
static_assert(sizeof(LocationTimepoint) == 8);
static_assert(std::is_trivial<LocationTimepoint>::value);
inline bool operator==(const LocationTimepoint a, const LocationTimepoint b)
{
    return a.nt == b.nt;
}
inline bool operator!=(const LocationTimepoint a, const LocationTimepoint b)
{
    return a.nt != b.nt;
}

union EdgeTimepoint
{
    struct
    {
        Edge e;
        Timepoint t;
    } et;
    struct
    {
        Location n : 29;
        Direction d : 3;
        Timepoint t;
    };
    uint64_t id;

    EdgeTimepoint() noexcept = default;
    explicit EdgeTimepoint(const Edge e, const Timepoint t) noexcept : et{e, t} {}
    explicit EdgeTimepoint(const LocationTimepoint nt, const Direction d) noexcept : n{nt.n}, d{d}, t{nt.t} {}
    explicit EdgeTimepoint(const Location n, const Direction d, const Timepoint t) noexcept : n{n}, d{d}, t{t} {}

    inline LocationTimepoint nt() const noexcept { return LocationTimepoint{n, t}; }
};
static_assert(sizeof(EdgeTimepoint) == 8);
static_assert(std::is_trivial<EdgeTimepoint>::value);
inline bool operator==(const EdgeTimepoint a, const EdgeTimepoint b)
{
    return a.id == b.id;
}
inline bool operator!=(const EdgeTimepoint a, const EdgeTimepoint b)
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
struct hash<TruffleHog::LocationTimepoint>
{
    inline std::size_t operator()(const TruffleHog::LocationTimepoint nt) const noexcept
    {
        return robin_hood::hash<uint64_t>{}(nt.nt);
    }
};

template<>
struct hash<TruffleHog::EdgeTimepoint>
{
    inline std::size_t operator()(const TruffleHog::EdgeTimepoint et) const noexcept
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
struct formatter<TruffleHog::LocationTimepoint>
{
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx) { return ctx.begin(); }

    template<typename FormatContext>
    inline auto format(const TruffleHog::LocationTimepoint nt, FormatContext& ctx)
    {
        return format_to(ctx.out(), "(n={},t={})", nt.n, nt.t);
    }
};

template<>
struct formatter<TruffleHog::EdgeTimepoint>
{
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx) { return ctx.begin(); }

    template<typename FormatContext>
    inline auto format(const TruffleHog::EdgeTimepoint et, FormatContext& ctx)
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
