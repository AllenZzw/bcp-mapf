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

#ifndef MAPF_SEPARATOR_H
#define MAPF_SEPARATOR_H

#include "Includes.h"
#include "Coordinates.h"

struct RobotRobustCut
{
    SCIP_ROW* row;
    const EdgeTimepoint* begin;
    const EdgeTimepoint* end;
};

class TwoRobotRobustCut
{
#ifdef DEBUG
    String name_;
#endif
    SCIP_ROW* row_;
    Robot a1_;
    Robot a2_;
    Int a1_end;
    Int a2_end_;
    EdgeTimepoint* ets_;

  public:
    // Constructors
    TwoRobotRobustCut(
        SCIP* scip,
        const Robot a1,
        const Robot a2,
        const Int nb_a1_edgetimes,
        const Int nb_a2_edgetimes
#ifdef DEBUG
      , String&& name
#endif
    ) :
#ifdef DEBUG
        name_(std::move(name)),
#endif
        row_(nullptr),
        a1_(a1),
        a2_(a2),
        a1_end(nb_a1_edgetimes),
        a2_end_(nb_a1_edgetimes + nb_a2_edgetimes)
    {
        scip_assert(SCIPallocBlockMemoryArray(scip, &ets_, a2_end_));
    }

    // Getters
#ifdef DEBUG
    inline const auto& name() const { return name_; }
#endif
    inline auto row() const { return row_; }
    inline auto a1() const { return a1_; }
    inline auto a2() const { return a2_; }
    inline auto begin() const { return ets_; }
    inline auto size() const { return a2_end_; }
    inline const EdgeTimepoint* a1_edge_times_begin() const { return &ets_[0]; }
    inline const EdgeTimepoint* a1_edge_times_end() const { return &ets_[a1_end]; }
    inline const EdgeTimepoint* a2_edge_times_begin() const { return &ets_[a1_end]; }
    inline const EdgeTimepoint* a2_edge_times_end() const { return &ets_[a2_end_]; }
    inline Pair<const EdgeTimepoint*, const EdgeTimepoint*> a1_edge_times() const
    {
        return {a1_edge_times_begin(), a1_edge_times_end()};
    }
    inline Pair<const EdgeTimepoint*, const EdgeTimepoint*> a2_edge_times() const
    {
        return {a2_edge_times_begin(), a2_edge_times_end()};
    }
    inline Array<Tuple<Robot, const EdgeTimepoint*, const EdgeTimepoint*>, 2> iterators() const
    {
        using T = Tuple<Robot, const EdgeTimepoint*, const EdgeTimepoint*>;
        return {T{a1(), a1_edge_times_begin(), a1_edge_times_end()},
                T{a2(), a2_edge_times_begin(), a2_edge_times_end()}};
    };
    inline Pair<const EdgeTimepoint*, const EdgeTimepoint*> edge_times(const Robot a) const
    {
        debug_assert(a == a1_ || a == a2_);
        return a == a1_ ? a1_edge_times() : a2_edge_times();
    }

    // Setters
    inline void set_row(SCIP_ROW* row) { row_ = row; }
    inline EdgeTimepoint& a1_edge_time(const Int idx) { return ets_[idx]; }
    inline EdgeTimepoint& a2_edge_time(const Int idx) { return ets_[a1_end + idx]; }
};

#endif
