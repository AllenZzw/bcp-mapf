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

//#define PRINT_DEBUG

#include "AStar.h"
#include <cstddef>

#define EPS (1e-6)
#define isEQ(x, y) (std::abs((x)-(y)) <= (EPS))
#define isLT(x, y) ((x)-(y) < -(EPS))
#define isLE(x, y) ((x)-(y) <= (EPS))
#define isGT(x, y) ((x)-(y) > (EPS))
#define isGE(x, y) ((x)-(y) >= -(EPS))

namespace TruffleHog
{

#ifdef DEBUG
static bool verbose = false;
#endif

static inline bool get_bitset(const std::byte* const bitset, const Int i)
{
    const auto idx = i / CHAR_BIT;
    const auto mask = std::byte(0x01) << (i % CHAR_BIT);
    return (bitset[idx] & mask) != std::byte(0x0);
}

static inline void set_bitset(std::byte* const bitset, const Int i)
{
    const auto idx = i / CHAR_BIT;
    const auto mask = std::byte(0x01) << (i % CHAR_BIT);
    bitset[idx] |= mask;
}

//static inline void clear_bitset(std::byte* const bitset, const Int i)
//{
//    const auto idx = i / CHAR_BIT;
//    const auto mask = std::byte(0x01) << (i % CHAR_BIT);
//    bitset[idx] &= ~mask;
//}

#ifdef DEBUG
String make_goal_state_string(const std::byte* const state, const Int nb_goal_crossings)
{
    String str;
    if (nb_goal_crossings > 0)
    {
        str = ", goal ";
        for (Int idx = 0; idx < nb_goal_crossings; ++idx)
        {
            str.push_back('0' + static_cast<char>(get_bitset(state, idx)));
        }
    }
    return str;
}
#endif


void AStar::Data::smooth(const Data& previous_data, const double alpha) 
{
    cost_offset = alpha * previous_data.cost_offset + (1 - alpha) * cost_offset; 

    for (const auto& [nt, previous_edge_costs] : previous_data.edge_penalties)
        if (previous_edge_costs.used)
        {
            auto& current_edge_costs = edge_penalties.get_edge_penalties(nt);
            current_edge_costs.north = alpha * previous_edge_costs.north + (1 - alpha) * current_edge_costs.north; 
            current_edge_costs.south = alpha * previous_edge_costs.south + (1 - alpha) * current_edge_costs.south; 
            current_edge_costs.east = alpha * previous_edge_costs.east + (1 - alpha) * current_edge_costs.east; 
            current_edge_costs.west = alpha * previous_edge_costs.west + (1 - alpha) * current_edge_costs.west; 
            current_edge_costs.wait = alpha * previous_edge_costs.wait + (1 - alpha) * current_edge_costs.wait; 
        }

    for (auto& cost: finish_time_penalties) 
    {
        cost = (1 - alpha) * cost; 
    }

    for (Timepoint t = 0; t < static_cast<Timepoint>(previous_data.finish_time_penalties.size()); ++t)
    {
        finish_time_penalties.add(t, alpha * previous_data.finish_time_penalties[t] ); 
    }

#ifdef USE_GOAL_CONFLICTS
    for (auto& goal_penalty: goal_penalties)
        goal_penalty.cost = (1 - alpha) * goal_penalty.cost; 
    
    for (Int idx = 0; idx < static_cast<Int>(previous_data.goal_penalties.size()); ++idx)
        goal_penalties.add(previous_data.goal_penalties[idx].nt, alpha * previous_data.goal_penalties[idx].cost); 
#endif
}

bool AStar::Data::can_be_better(const Data& previous_data)
{
    if (cost_offset < previous_data.cost_offset ||
        waypoints != previous_data.waypoints ||
        latest_goal_time != previous_data.latest_goal_time ||
        earliest_goal_time != previous_data.earliest_goal_time)
    {
        return true;
    }

    for (const auto& [nt, previous_edge_costs] : previous_data.edge_penalties)
        if (previous_edge_costs.used)
        {
            const auto it = edge_penalties.find(nt);
            if (it == edge_penalties.end())
            {
                return true;
            }
            const auto& current_edge_costs = it->second;
            if (current_edge_costs.north < previous_edge_costs.north ||
                current_edge_costs.south < previous_edge_costs.south ||
                current_edge_costs.east < previous_edge_costs.east ||
                current_edge_costs.west < previous_edge_costs.west ||
                current_edge_costs.wait < previous_edge_costs.wait)
            {
                return true;
            }
        }

    debug_assert(latest_visit_time.size() == previous_data.latest_visit_time.size());
    for (Location n = 0; n < static_cast<Location>(latest_visit_time.size()); ++n)
        if (latest_visit_time[n] > previous_data.latest_visit_time[n])
        {
            return true;
        }

    if (finish_time_penalties.size() != previous_data.finish_time_penalties.size())
    {
        return true;
    }
    for (Timepoint t = 0; t < static_cast<Timepoint>(finish_time_penalties.size()); ++t)
        if (finish_time_penalties[t] < previous_data.finish_time_penalties[t])
        {
            return true;
        }

#ifdef USE_GOAL_CONFLICTS
    if (goal_penalties.size() != previous_data.goal_penalties.size())
    {
        return true;
    }
    for (Int idx = 0; idx < static_cast<Int>(goal_penalties.size()); ++idx)
        if (goal_penalties[idx].nt != previous_data.goal_penalties[idx].nt ||
            goal_penalties[idx].cost < previous_data.goal_penalties[idx].cost)
        {
            return true;
        }
#endif

    return false;
}

AStar::AStar(const Map& map) :
    map_(map),

    data_(),

    h_node_to_waypoint_(nullptr),
    h_waypoint_to_goal_(),
    heuristic_(map),
    label_pool_(),
#ifdef USE_RESERVATION_TABLE
    open_(map.size()),
#else
    open_(),
#endif
    frontier_()
#ifdef DEBUG
  , nb_labels_(0)
#endif
{
}

AStar::Label* AStar::dominated(Label* const new_label)
{
    // Check.
//#ifdef DEBUG
//    for (auto it = frontier_.begin(); it != frontier_.end(); ++it)
//    {
//        auto label = it->second;
//        open_.check_label(label);
//    }
//#endif

    // Try to put in the new label.
    auto [it, success] = frontier_.try_emplace(LocationTimepoint{new_label->nt}, new_label);

    // Check for dominance if a label already exists.
    if (!success)
    {
        auto existing_label = it->second;
        debug_assert(existing_label->nt == new_label->nt);
        if (isLE(existing_label->f, new_label->f))
        {
            // Existing label dominates new label.
            debug_assert(isLE(existing_label->g, new_label->g));

            // Dominated.
            return nullptr;
        }
        else
        {
            // New label dominates existing label.
            debug_assert(it == frontier_.find(new_label->nt));
            debug_assert(!isLE(existing_label->g, new_label->g));

            // If the existing label has already been popped, push the new label. Otherwise, replace
            // the existing label with the new label.
            if (existing_label->pqueue_index >= 0)
            {
                open_.update_pqueue_index(new_label, existing_label->pqueue_index);
                memcpy(existing_label, new_label, label_pool_.label_size());
                open_.decrease_key(existing_label);
            }
            else
            {
                debug_assert(new_label->pqueue_index == -1);
                memcpy(existing_label, new_label, label_pool_.label_size());
                open_.push(existing_label);
            }
            return existing_label;
        }
    }
    else
    {
        // Store the label.
        label_pool_.commit_latest_label();
        open_.push(new_label);

        // Not dominated.
        return new_label;
    }
}

void AStar::generate_start()
{
    // Get data.
    const auto& [start, 
                 waypoints, 
                 goal, 
                 earliest_goal_time, 
                 latest_goal_time, 
                 cost_offset, 
                 latest_visit_time,
                 edge_penalties, 
                 finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
               , goal_penalties
#endif
    ] = data_;
    constexpr auto start_time = 0;
    const auto waypoint_time = waypoints[0].t;

    // Create label.
    auto new_label = reinterpret_cast<Label*>(label_pool_.get_label_buffer());
    memset(new_label, 0, label_pool_.label_size());
#ifdef DEBUG
    new_label->label_id = nb_labels_++;
#endif
    const auto h_node_to_waypoint = std::max((*h_node_to_waypoint_)[start], waypoint_time - start_time);
    const auto h_waypoint_to_goal = h_waypoint_to_goal_[0];
    const auto h_goal_to_finish = finish_time_penalties.get_h(start_time + h_node_to_waypoint + h_waypoint_to_goal);
    const auto h = std::max(h_node_to_waypoint, waypoint_time - start_time) + h_waypoint_to_goal + h_goal_to_finish;
    debug_assert(h_node_to_waypoint >= 0);
    debug_assert(h_waypoint_to_goal >= 0);
    debug_assert(h_goal_to_finish >= 0);
    new_label->g = cost_offset;
    new_label->f = cost_offset + h;
    new_label->nt = LocationTimepoint{start, start_time}.nt;

    // Store the label.
    debug_assert(open_.empty());
    debug_assert(frontier_.empty());
    dominated(new_label);

    // Print.
#ifdef DEBUG
    if (verbose)
    {
#ifdef USE_GOAL_CONFLICTS
        const auto nb_goal_penalties = goal_penalties.size();
#else
        const auto nb_goal_penalties = 0;
#endif
        println("    Generating start label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
                new_label->label_id,
                fmt::ptr(new_label),
                decltype(new_label->n){new_label->n},
                decltype(new_label->t){new_label->t},
                decltype(new_label->nt){new_label->nt},
                map_.get_x(new_label->n),
                map_.get_y(new_label->n),
                new_label->g,
                h,
                new_label->f,
                make_goal_state_string(&new_label->state_[0], nb_goal_penalties));
    }
#endif
}

void AStar::generate_end(Label* const current)
{
    // Get data.
    const auto& [start, 
                 waypoints, 
                 goal, 
                 earliest_goal_time, 
                 latest_goal_time, 
                 cost_offset, 
                 latest_visit_time,
                 edge_penalties, 
                 finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
               , goal_penalties
#endif
    ] = data_;

    // Create label.
    auto new_label = reinterpret_cast<Label*>(label_pool_.get_label_buffer());
    memcpy(new_label, current, label_pool_.label_size());
#ifdef DEBUG
    new_label->label_id = nb_labels_++;
#endif
    new_label->parent = current;
    debug_assert((*h_node_to_waypoint_)[current->n] == 0);
    new_label->g += finish_time_penalties.get_penalty(current->t);
    new_label->f = new_label->g;
    new_label->n = -1;

    // Check if cost-infeasible.
    if (isGE(new_label->f, 0))
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
#ifdef USE_GOAL_CONFLICTS
            const auto nb_goal_penalties = goal_penalties.size();
#else
            const auto nb_goal_penalties = 0;
#endif
            println("    Cost-infeasible end label {} {} (t {}, g {}{})",
                    new_label->label_id,
                    fmt::ptr(new_label),
                    decltype(new_label->t){new_label->t},
                    new_label->g,
                    make_goal_state_string(&new_label->state_[0], nb_goal_penalties));
        }
#endif

        // Done.
        return;
    }

    // Store the label.
    label_pool_.commit_latest_label();
    open_.push(new_label);

    // Print.
#ifdef DEBUG
    if (verbose)
    {
#ifdef USE_GOAL_CONFLICTS
        const auto nb_goal_penalties = goal_penalties.size();
#else
        const auto nb_goal_penalties = 0;
#endif
        println("    Generating end label {} {} (t {}, g {}{})",
                new_label->label_id,
                fmt::ptr(new_label),
                decltype(new_label->t){new_label->t},
                new_label->g,
                make_goal_state_string(&new_label->state_[0], nb_goal_penalties));
    }
#endif
}

void AStar::generate(Label* const current,
                     const Waypoint w,
                     const Location next_n,
                     const Cost cost,
                     const Timepoint waypoint_time)
{
    // Get data.
    const auto& [start, 
                 waypoints, 
                 goal, 
                 earliest_goal_time, 
                 latest_goal_time, 
                 cost_offset, 
                 latest_visit_time,
                 edge_penalties, 
                 finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
               , goal_penalties
#endif
    ] = data_;

    // Compute the vertex (node-time) of the new label.
    const auto next_t = current->t + 1;
    const LocationTimepoint next_nt{next_n, next_t};

    // Check if time-infeasible.
    const auto h_node_to_waypoint = std::max((*h_node_to_waypoint_)[next_nt.n], waypoint_time - next_t);
    const auto h_waypoint_to_goal = h_waypoint_to_goal_[w];
    debug_assert(h_node_to_waypoint >= 0);
    debug_assert(h_waypoint_to_goal >= 0);
    if (next_t + h_node_to_waypoint > waypoint_time || next_t + h_node_to_waypoint + h_waypoint_to_goal > latest_goal_time)
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
            println("    Timepoint-infeasible label (n {}, t {}, nt {}, xy ({},{}))",
                    next_nt.n,
                    next_nt.t,
                    next_nt.nt,
                    map_.get_x(next_nt.n),
                    map_.get_y(next_nt.n));
        }
#endif

        // Done.
        return;
    }

    // Create label.
    auto next_label = reinterpret_cast<Label*>(label_pool_.get_label_buffer());
    memcpy(next_label, current, label_pool_.label_size());
#ifdef DEBUG
    next_label->label_id = nb_labels_++;
#endif
    next_label->parent = current;
    next_label->g = current->g + cost;
    next_label->nt = next_nt.nt;
#ifdef USE_RESERVATION_TABLE
    next_label->reserved = reservation_table().is_reserved(next_nt);
#endif

    // Check all goal crossings.
#ifdef USE_GOAL_CONFLICTS
    for (Int idx = 0; idx < goal_penalties.size(); ++idx)
    {
        const auto [goal_nt, goal_cost] = goal_penalties[idx];

        const auto crossed = get_bitset(next_label->state_, idx);
        if (!crossed && next_n == goal_nt.n && next_t >= goal_nt.t)
        {
            // Incur the penalty.
            next_label->g += goal_cost;
            set_bitset(next_label->state_, idx);
        }
    }
#endif

    // Compute f.
    const auto h_goal_to_finish = finish_time_penalties.get_h(next_t + h_node_to_waypoint + h_waypoint_to_goal);
    const auto h = std::max(h_node_to_waypoint, waypoint_time - next_t) + h_waypoint_to_goal + h_goal_to_finish;
    next_label->f = next_label->g + h;
    debug_assert(isGE(next_label->g, current->g + 1));
    debug_assert(isGE(next_label->f, current->f));

    // Check if cost-infeasible.
    if (isGE(next_label->f, 0))
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
#ifdef USE_GOAL_CONFLICTS
            const auto nb_goal_penalties = goal_penalties.size();
#else
            const auto nb_goal_penalties = 0;
#endif
            println("    Cost-infeasible label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
                    next_label->label_id,
                    fmt::ptr(next_label),
                    decltype(next_label->n){next_label->n},
                    decltype(next_label->t){next_label->t},
                    decltype(next_label->nt){next_label->nt},
                    map_.get_x(next_label->n),
                    map_.get_y(next_label->n),
                    next_label->g,
                    h,
                    next_label->f,
                    make_goal_state_string(&next_label->state_[0], nb_goal_penalties));
        }
#endif

        // Done.
        return;
    }

    // Store the label if not dominated.
#ifdef DEBUG
    auto next_label_copy = next_label;
#endif
    next_label = dominated(next_label);

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        if (next_label)
        {
#ifdef USE_GOAL_CONFLICTS
            const auto nb_goal_penalties = goal_penalties.size();
#else
            const auto nb_goal_penalties = 0;
#endif
            println("    Generating label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
                    next_label_copy->label_id,
                    fmt::ptr(next_label_copy),
                    decltype(next_label_copy->n){next_label_copy->n},
                    decltype(next_label_copy->t){next_label_copy->t},
                    decltype(next_label_copy->nt){next_label_copy->nt},
                    map_.get_x(next_label_copy->n),
                    map_.get_y(next_label_copy->n),
                    next_label_copy->g,
                    h,
                    next_label_copy->f,
                    make_goal_state_string(&next_label_copy->state_[0], nb_goal_penalties));
        }
        else
        {
#ifdef USE_GOAL_CONFLICTS
            const auto nb_goal_penalties = goal_penalties.size();
#else
            const auto nb_goal_penalties = 0;
#endif
            println("    Dominated label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
                    next_label_copy->label_id,
                    fmt::ptr(next_label_copy),
                    decltype(next_label_copy->n){next_label_copy->n},
                    decltype(next_label_copy->t){next_label_copy->t},
                    decltype(next_label_copy->nt){next_label_copy->nt},
                    map_.get_x(next_label_copy->n),
                    map_.get_y(next_label_copy->n),
                    next_label_copy->g,
                    h,
                    next_label_copy->f,
                    make_goal_state_string(&next_label_copy->state_[0], nb_goal_penalties));
        }
    }
#endif
}

void AStar::generate_last_segment(Label* const current,
                                  const Location next_n,
                                  const Cost cost)
{
    // Get data.
    const auto& [start, 
                 waypoints, 
                 goal, 
                 earliest_goal_time, 
                 latest_goal_time, 
                 cost_offset, 
                 latest_visit_time,
                 edge_penalties, 
                 finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
               , goal_penalties
#endif
    ] = data_;

    // Compute the vertex (node-time) of the new label.
    const auto next_t = current->t + 1;
    const LocationTimepoint next_nt{next_n, next_t};

    // Check if time-infeasible.
    const auto h_node_to_waypoint = std::max((*h_node_to_waypoint_)[next_nt.n], earliest_goal_time - next_t);
    debug_assert(h_node_to_waypoint >= 0);
    if (next_t + h_node_to_waypoint > latest_goal_time)
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
            println("    Timepoint-infeasible label (n {}, t {}, nt {}, xy ({},{}))",
                    next_nt.n,
                    next_nt.t,
                    next_nt.nt,
                    map_.get_x(next_nt.n),
                    map_.get_y(next_nt.n));
        }
#endif

        // Done.
        return;
    }

    // Create label.
    auto next_label = reinterpret_cast<Label*>(label_pool_.get_label_buffer());
    memcpy(next_label, current, label_pool_.label_size());
#ifdef DEBUG
    next_label->label_id = nb_labels_++;
#endif
    next_label->parent = current;
    next_label->g = current->g + cost;
    next_label->nt = next_nt.nt;
#ifdef USE_RESERVATION_TABLE
    next_label->reserved = reservation_table().is_reserved(next_nt);
#endif

    // Check all goal crossings.
#ifdef USE_GOAL_CONFLICTS
    for (Int idx = 0; idx < goal_penalties.size(); ++idx)
    {
        const auto [goal_nt, goal_cost] = goal_penalties[idx];

        const auto crossed = get_bitset(next_label->state_, idx);
        if (!crossed && next_n == goal_nt.n && next_t >= goal_nt.t)
        {
            // Incur the penalty.
            next_label->g += goal_cost;
            set_bitset(next_label->state_, idx);
        }
    }
#endif

    // Compute f.
    const auto h_goal_to_finish = finish_time_penalties.get_h(next_t + h_node_to_waypoint);
    const auto h = std::max(h_node_to_waypoint, earliest_goal_time - next_t) + h_goal_to_finish;
    next_label->f = next_label->g + h;
    debug_assert(isGE(next_label->g, current->g + 1));
    debug_assert(isGE(next_label->f, current->f));

    // Check if cost-infeasible.
    if (isGE(next_label->f, 0))
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
#ifdef USE_GOAL_CONFLICTS
            const auto nb_goal_penalties = goal_penalties.size();
#else
            const auto nb_goal_penalties = 0;
#endif
            println("    Cost-infeasible label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
                    next_label->label_id,
                    fmt::ptr(next_label),
                    decltype(next_label->n){next_label->n},
                    decltype(next_label->t){next_label->t},
                    decltype(next_label->nt){next_label->nt},
                    map_.get_x(next_label->n),
                    map_.get_y(next_label->n),
                    next_label->g,
                    h,
                    next_label->f,
                    make_goal_state_string(&next_label->state_[0], nb_goal_penalties));
        }
#endif

        // Done.
        return;
    }

    // Store the label if not dominated.
#ifdef DEBUG
    auto next_label_copy = next_label;
#endif
    next_label = dominated(next_label);

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        if (next_label)
        {
#ifdef USE_GOAL_CONFLICTS
            const auto nb_goal_penalties = goal_penalties.size();
#else
            const auto nb_goal_penalties = 0;
#endif
            println("    Generating label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
                    next_label_copy->label_id,
                    fmt::ptr(next_label_copy),
                    decltype(next_label_copy->n){next_label_copy->n},
                    decltype(next_label_copy->t){next_label_copy->t},
                    decltype(next_label_copy->nt){next_label_copy->nt},
                    map_.get_x(next_label_copy->n),
                    map_.get_y(next_label_copy->n),
                    next_label_copy->g,
                    h,
                    next_label_copy->f,
                    make_goal_state_string(&next_label_copy->state_[0], nb_goal_penalties));
        }
        else
        {
#ifdef USE_GOAL_CONFLICTS
            const auto nb_goal_penalties = goal_penalties.size();
#else
            const auto nb_goal_penalties = 0;
#endif
            println("    Dominated label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
                    next_label_copy->label_id,
                    fmt::ptr(next_label_copy),
                    decltype(next_label_copy->n){next_label_copy->n},
                    decltype(next_label_copy->t){next_label_copy->t},
                    decltype(next_label_copy->nt){next_label_copy->nt},
                    map_.get_x(next_label_copy->n),
                    map_.get_y(next_label_copy->n),
                    next_label_copy->g,
                    h,
                    next_label_copy->f,
                    make_goal_state_string(&next_label_copy->state_[0], nb_goal_penalties));
        }
    }
#endif
}

template<IntCost default_cost>
void AStar::generate_neighbours(Label* const current, const Waypoint w, const Timepoint waypoint_time)
{
    // Get data.
    auto& [start, 
           waypoints, 
           goal, 
           earliest_goal_time, 
           latest_goal_time, 
           cost_offset, 
           latest_visit_time,
           edge_penalties, 
           finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
         , goal_penalties
#endif
    ] = data_;

    // Print.
#ifdef DEBUG
    if (verbose)
    {
#ifdef USE_GOAL_CONFLICTS
        const auto nb_goal_penalties = goal_penalties.size();
#else
        const auto nb_goal_penalties = 0;
#endif
        println("Expanding label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
                current->label_id,
                fmt::ptr(current),
                decltype(current->n){current->n},
                decltype(current->t){current->t},
                decltype(current->nt){current->nt},
                map_.get_x(current->n),
                map_.get_y(current->n),
                current->g,
                current->f - current->g,
                current->f,
                make_goal_state_string(&current->state_[0], nb_goal_penalties));
    }
#endif

    // Expand in five directions.
    const auto edge_costs = edge_penalties.get_edge_costs<default_cost>(current->nt);
    const auto current_n = current->n;
    if (const auto next_n = map_.get_north(current_n);
        latest_visit_time[next_n] >= current->t + 1 && edge_costs.north < std::numeric_limits<Cost>::infinity())
    {
        generate(current, w, next_n, edge_costs.north, waypoint_time);
    }
    if (const auto next_n = map_.get_south(current_n);
        latest_visit_time[next_n] >= current->t + 1 && edge_costs.south < std::numeric_limits<Cost>::infinity())
    {
        generate(current, w, next_n, edge_costs.south, waypoint_time);
    }
    if (const auto next_n = map_.get_east(current_n);
        latest_visit_time[next_n] >= current->t + 1 && edge_costs.east < std::numeric_limits<Cost>::infinity())
    {
        generate(current, w, next_n, edge_costs.east, waypoint_time);
    }
    if (const auto next_n = map_.get_west(current_n);
        latest_visit_time[next_n] >= current->t + 1 && edge_costs.west < std::numeric_limits<Cost>::infinity())
    {
        generate(current, w, next_n, edge_costs.west, waypoint_time);
    }
    if (const auto next_n = map_.get_wait(current_n);
        latest_visit_time[next_n] >= current->t + 1 && edge_costs.wait < std::numeric_limits<Cost>::infinity())
    {
        generate(current, w, next_n, edge_costs.wait, waypoint_time);
    }
}
template
void AStar::generate_neighbours<0>(Label* const current,
                                   const Waypoint w,
                                   const Timepoint waypoint_time);
template
void AStar::generate_neighbours<1>(Label* const current,
                                   const Waypoint w,
                                   const Timepoint waypoint_time);

template<IntCost default_cost>
void AStar::generate_neighbours_last_segment(Label* const current)
{
    // Get data.
    auto& [start, 
           waypoints, 
           goal, 
           earliest_goal_time, 
           latest_goal_time, 
           cost_offset, 
           latest_visit_time,
           edge_penalties, 
           finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
         , goal_penalties
#endif
    ] = data_;

    // Print.
#ifdef DEBUG
    if (verbose)
    {
#ifdef USE_GOAL_CONFLICTS
        const auto nb_goal_penalties = goal_penalties.size();
#else
        const auto nb_goal_penalties = 0;
#endif
        println("Expanding label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
                current->label_id,
                fmt::ptr(current),
                decltype(current->n){current->n},
                decltype(current->t){current->t},
                decltype(current->nt){current->nt},
                map_.get_x(current->n),
                map_.get_y(current->n),
                current->g,
                current->f - current->g,
                current->f,
                make_goal_state_string(&current->state_[0], nb_goal_penalties));
    }
#endif

    // Expand in five directions.
    const auto edge_costs = edge_penalties.get_edge_costs<default_cost>(current->nt);
    const auto current_n = current->n;
    if (const auto next_n = map_.get_north(current_n);
        latest_visit_time[next_n] >= current->t + 1 && edge_costs.north < std::numeric_limits<Cost>::infinity())
    {
        generate_last_segment(current, next_n, edge_costs.north);
    }
    if (const auto next_n = map_.get_south(current_n);
        latest_visit_time[next_n] >= current->t + 1 && edge_costs.south < std::numeric_limits<Cost>::infinity())
    {
        generate_last_segment(current, next_n, edge_costs.south);
    }
    if (const auto next_n = map_.get_east(current_n);
        latest_visit_time[next_n] >= current->t + 1 && edge_costs.east < std::numeric_limits<Cost>::infinity())
    {
        generate_last_segment(current, next_n, edge_costs.east);
    }
    if (const auto next_n = map_.get_west(current_n);
        latest_visit_time[next_n] >= current->t + 1 && edge_costs.west < std::numeric_limits<Cost>::infinity())
    {
        generate_last_segment(current, next_n, edge_costs.west);
    }
    if (const auto next_n = map_.get_wait(current_n);
        latest_visit_time[next_n] >= current->t + 1 && edge_costs.wait < std::numeric_limits<Cost>::infinity())
    {
        generate_last_segment(current, next_n, edge_costs.wait);
    }
}
template
void AStar::generate_neighbours_last_segment<0>(Label* const current);
template
void AStar::generate_neighbours_last_segment<1>(Label* const current);

void AStar::preprocess_input()
{
    // Get data.
    auto& [start, 
           waypoints, 
           goal, 
           earliest_goal_time, 
           latest_goal_time, 
           cost_offset, 
           latest_visit_time,
           edge_penalties, 
           finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
         , goal_penalties
#endif
    ] = data_;

    // Append the goal as a waypoint.
    debug_assert(earliest_goal_time <= latest_goal_time);
    waypoints.push_back(LocationTimepoint{goal, earliest_goal_time});
}

template<bool is_farkas>
Pair<Vector<LocationTimepoint>, Cost> AStar::solve()
{
    // Get data.
    const auto& [start, 
                 waypoints, 
                 goal, 
                 earliest_goal_time, 
                 latest_goal_time, 
                 cost_offset, 
                 latest_visit_time,
                 edge_penalties, 
                 finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
               , goal_penalties
#endif
    ] = data_;

    // Print.
#ifdef PRINT_DEBUG
    {
        String str;
        for (auto it = waypoints.begin(); it != waypoints.end() - 1; ++it)
        {
            if (it == waypoints.begin())
            {
                str += " via " ;
            }
            else
            {
                str += ", ";
            }
            str += fmt::format("(({},{}),{})", map_.get_x(it->n), map_.get_y(it->n), it->t);
        }
        println("Solving from ({},{}){} to ({},{}) between times {} and {}",
                map_.get_x(start),
                map_.get_y(start),
                str,
                map_.get_x(goal),
                map_.get_y(goal),
                earliest_goal_time,
                latest_goal_time);
    }
#endif

    // Create output.
    Pair<Vector<LocationTimepoint>, Cost> output;
    auto& path = output.first;
    auto& path_cost = output.second;

    // Prepare costs.
    data_.edge_penalties.before_solve();
    data_.finish_time_penalties.before_solve();
#ifdef USE_GOAL_CONFLICTS
    data_.goal_penalties.before_solve();
#endif

    // Get number of resources.
#ifdef USE_GOAL_CONFLICTS
    const auto nb_goal_crossings = goal_penalties.size();
#else
    constexpr Int nb_goal_crossings = 0;
#endif

    // Reset.
    const auto nb_states = nb_goal_crossings;
    label_pool_.reset(sizeof(Label) + (nb_states + CHAR_BIT - 1) / CHAR_BIT);
    open_.clear();
    frontier_.clear();

    // Compute minimum time between each waypoint.
    h_waypoint_to_goal_.resize(waypoints.size());
    h_waypoint_to_goal_.back() = 0;
    for (Waypoint w = waypoints.size() - 2; w >= 0; --w)
    {
        const auto h = heuristic_.get_h(waypoints[w + 1].n)[waypoints[w].n];
        const auto t_diff = waypoints[w + 1].t - waypoints[w].t;
        if (w != static_cast<Waypoint>(waypoints.size() - 2) && t_diff < h)
        {
            return output;
        }
        h_waypoint_to_goal_[w] = std::max(h, t_diff) + h_waypoint_to_goal_[w + 1];
    }

    // Create the first label.
    Waypoint w = 0;
    h_node_to_waypoint_ = &heuristic_.get_h(waypoints[w].n);
    generate_start();

    // Solve up to but not including the last waypoint (goal).
    constexpr IntCost default_cost = is_farkas ? 0 : 1;
    if (waypoints.size() > 1)
    {
        while (!open_.empty())
        {
            // Get a label from priority queue.
            const auto current = open_.top();
            open_.pop();

            // Advance to the next waypoint.
            debug_assert(current->t <= waypoints[w].t);
            if (current->nt == waypoints[w])
            {
                // Print.
#ifdef DEBUG
                if (verbose)
                {
                    // Store the path.
                    Vector<LocationTimepoint> path;
                    for (auto l = current; l; l = l->parent)
                    {
                        path.push_back(l->nt);
                    }
                    std::reverse(path.begin(), path.end());

                    // Print.
#ifdef USE_GOAL_CONFLICTS
                    const auto nb_goal_penalties = goal_penalties.size();
#else
                    const auto nb_goal_penalties = 0;
#endif
                    fmt::print("Reached waypoint at label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{}) "
                               "with path",
                               current->label_id,
                               fmt::ptr(current),
                               decltype(current->n){current->n},
                               decltype(current->t){current->t},
                               decltype(current->nt){current->nt},
                               map_.get_x(current->n),
                               map_.get_y(current->n),
                               current->g,
                               current->f - current->g,
                               current->f,
                               make_goal_state_string(&current->state_[0], nb_goal_penalties));
                    for (const auto nt: path)
                    {
                        fmt::print(" ({},{})", map_.get_x(nt.n), map_.get_y(nt.n));
                    }
                    println("");
                }
#endif

                // Advance to the next waypoint.
                ++w;
                h_node_to_waypoint_ = &heuristic_.get_h(waypoints[w].n);

                // Clear priority queue.
                open_.clear();

                // Stop if reached the last waypoint (goal).
                if (w == static_cast<Waypoint>(waypoints.size() - 1))
                {
                    open_.push(current);
                    break;
                }
            }

            // Generate neighbours.
            generate_neighbours<default_cost>(current, w, waypoints[w].t);
        }
    }

    // Solve the last segment.
    while (!open_.empty())
    {
        // Get a label from priority queue.
        const auto current = open_.top();
        open_.pop();

        // Expand the neighbours of the current label or exit if the goal is reached.
        debug_assert(current->t <= latest_goal_time);
        if (current->n >= 0)
        {
            // Generate neighbours.
            generate_neighbours_last_segment<default_cost>(current);

            // Generate to the end.
            if (current->n == goal && current->t >= earliest_goal_time)
            {
                generate_end(current);
            }
        }
        else
        {
            // Store the path cost.
            path_cost = current->g;

            // Store the path.
            debug_assert(path.empty());
            for (auto l = current->parent; l; l = l->parent)
            {
                path.push_back(l->nt);
            }
            std::reverse(path.begin(), path.end());

            // Print.
#ifdef DEBUG
            if (verbose)
            {
                println("Reached end at label {} {} (t {}, g {}{})",
                        current->label_id,
                        fmt::ptr(current),
                        decltype(current->t){current->t},
                        current->g,
                        make_goal_state_string(&current->state_[0], nb_goal_crossings));

                fmt::print("Found path with cost {}: ", path_cost);
                for (const auto nt : path)
                {
                    fmt::print("({},{}) ", map_.get_x(nt.n), map_.get_y(nt.n));
                }
                println("");
            }
#endif

            // Check.
            debug_assert(isLT(path_cost, 0));
            debug_assert(earliest_goal_time <= current->t && current->t <= latest_goal_time);

            // Finish.
            break;
        }
    }

    // Retrieve path.
#ifdef DEBUG
    if (verbose)
    {
        println("=======================================");
    }
#endif

    // Return.
    return output;
}
template Pair<Vector<LocationTimepoint>, Cost> AStar::solve<false>();
template Pair<Vector<LocationTimepoint>, Cost> AStar::solve<true>();

#ifdef DEBUG
Pair<Vector<LocationTimepoint>, Cost> AStar::calculate_cost(const Vector<Edge>& input_path)
{
    static Int iter = 0;
#ifdef DEBUG
    if (verbose)
    {
        println("=========================================================");
        println("START DEBUG ITER {}", iter);
    }
#endif
    iter++;

//    const LocationTimepoint start{input_path.front().n, 0};
//    const auto goal = input_path.back().n;
//    const Int goal_earliest = input_path.size() - 1;
//    const Int goal_latest = input_path.size() - 1;
//    const bool is_farkas = false;
//
//    const auto max_cost = std::numeric_limits<Cost>::max();

//    ------------------------------------

    // Create output.
    Pair<Vector<LocationTimepoint>, Cost> output;
//    auto& path = output.first;
//    auto& path_cost = output.second;
//
//    // Get h values to the goal node. Compute them if necessary.
//    debug_assert(heuristic_.max_path_length() >= 1);
//    h_ = &heuristic_.get_h(goal);
//
//    // Calculate the default edge cost.
//    constexpr IntCost default_cost = is_farkas ? 0 : 1;
//
//    // Remove vertices with default outgoing costs.
//    //    edge_duals_.clean_up(default_cost);
//
//    // Get number of resources.
//#ifdef USE_GOAL_CONFLICTS
//    const auto nb_goal_crossings = goal_penalties_.size();
//#else
//    constexpr Int nb_goal_crossings = 0;
//#endif
//#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
//    const auto nb_rect_crossings = static_cast<Int>(rectangle_crossings_.size());
//#else
//    constexpr Int nb_rect_crossings = 0;
//#endif
//
//    // Print.
//    if constexpr (without_resources)
//    {
//        debugln("Solving from ({},{}) at time {} to ({},{}) between times {} and {} "
//                "without resources",
//                map_.get_x(start.n),
//                map_.get_y(start.n),
//                start.t,
//                map_.get_x(goal),
//                map_.get_y(goal),
//                goal_earliest,
//                goal_latest);
//    }
//    else
//    {
//        debugln("Solving from ({},{}) at time {} to ({},{}) between times {} and {} with "
//                "{} rectangle resources",
//                map_.get_x(start.n),
//                map_.get_y(start.n),
//                start.t,
//                map_.get_x(goal),
//                map_.get_y(goal),
//                goal_earliest,
//                goal_latest,
//                nb_rect_crossings);
//    }
//
//    // Reset.
//    const auto nb_states = nb_goal_crossings + 2 * nb_rect_crossings;
//    label_pool_.reset(sizeof(Label) + (nb_states / CHAR_BIT) + (nb_states % CHAR_BIT != 0));
//    open_.clear();
//#ifndef USE_RECTANGLE_CLIQUE_CONFLICTS
//    static_assert(without_resources);
//#endif
//    if constexpr (without_resources)
//    {
//        frontier_.clear();
//    }
//#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
//    else
//    {
//        frontier_with_resources_.clear();
//    }
//#endif
//
//    // Prepare penalties before solving.
//    edge_penalties_.before_solve();
//    goal_penalties_.before_solve();
//    finish_time_penalties_.before_solve();
//
//    // Create label at the start node-time.
//    generate_start<without_resources>(start);
//
//    // Main loop.
//    Int idx = 1;
//    while (!open_.empty())
//    {
//        // Get a label from priority queue.
//        const auto current = open_.top();
//        open_.pop();
//
//        // Expand the neighbours of the current label or exit if the goal is reached.
//        if (current->n != -1)
//        {
//            // Generate neighbours.
//            {
//                // Get edge costs.
//                const auto edge_costs = edge_penalties_.get_edge_costs<default_cost>(current->nt);
//
//                // Expand in five directions.
//                const auto current_n = current->n;
//                debug_assert(edge_costs.north >= 0 && !std::isnan(edge_costs.north));
//                debug_assert(edge_costs.south >= 0 && !std::isnan(edge_costs.south));
//                debug_assert(edge_costs.east >= 0 && !std::isnan(edge_costs.east));
//                debug_assert(edge_costs.west >= 0 && !std::isnan(edge_costs.west));
//                debug_assert(edge_costs.wait >= 0 && !std::isnan(edge_costs.wait));
//                if (const auto next_n = map_.get_north(current_n);
//                    idx < static_cast<Int>(input_path.size()) && next_n == input_path[idx].n &&
//                    map_[next_n] && edge_costs.north < std::numeric_limits<Cost>::infinity())
//                {
//                    generate<without_resources>(current,
//                                                next_n,
//#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
//                                                Direction::NORTH,
//#endif
//                                                edge_costs.north,
//                                                goal_latest,
//                                                max_cost);
//                }
//                if (const auto next_n = map_.get_south(current_n);
//                    idx < static_cast<Int>(input_path.size()) && next_n == input_path[idx].n &&
//                    map_[next_n] && edge_costs.south < std::numeric_limits<Cost>::infinity())
//                {
//                    generate<without_resources>(current,
//                                                next_n,
//#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
//                                                Direction::SOUTH,
//#endif
//                                                edge_costs.south,
//                                                goal_latest,
//                                                max_cost);
//                }
//                if (const auto next_n = map_.get_east(current_n);
//                    idx < static_cast<Int>(input_path.size()) && next_n == input_path[idx].n &&
//                    map_[next_n] && edge_costs.east < std::numeric_limits<Cost>::infinity())
//                {
//                    generate<without_resources>(current,
//                                                next_n,
//#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
//                                                Direction::EAST,
//#endif
//                                                edge_costs.east,
//                                                goal_latest,
//                                                max_cost);
//                }
//                if (const auto next_n = map_.get_west(current_n);
//                    idx < static_cast<Int>(input_path.size()) && next_n == input_path[idx].n &&
//                    map_[next_n] && edge_costs.west < std::numeric_limits<Cost>::infinity())
//                {
//                    generate<without_resources>(current,
//                                                next_n,
//#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
//                                                Direction::WEST,
//#endif
//                                                edge_costs.west,
//                                                goal_latest,
//                                                max_cost);
//                }
//                if (const auto next_n = map_.get_wait(current_n);
//                    idx < static_cast<Int>(input_path.size()) && next_n == input_path[idx].n &&
//                    map_[next_n] && edge_costs.wait < std::numeric_limits<Cost>::infinity())
//                {
//                    generate<without_resources>(current,
//                                                next_n,
//#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
//                                                Direction::WAIT,
//#endif
//                                                edge_costs.wait,
//                                                goal_latest,
//                                                max_cost);
//                }
//
//                // Expand to the end dummy node.
//                if (current_n == goal && current->t >= goal_earliest)
//                {
//                    generate_end(current, max_cost);
//                }
//            }
//        }
//        else
//        {
//            // Get the label of the actual goal cell.
//            auto parent = current->parent;
//
//            // Store the path cost.
//            path_cost = current->g;
//
//            // Store the path.
//            for (auto l = parent; l; l = l->parent)
//            {
//                path.push_back(l->nt);
//            }
//            std::reverse(path.begin(), path.end());
//
//            // Print.
//#ifdef DEBUG
//            if (verbose)
//            {
//                println("Reached goal at label {} {} (n {}, t {}, nt {}, position ({},{}), g {}, h {}, f {}{}{})",
//                        current->label_id,
//                        fmt::ptr(current),
//                        decltype(parent->n){parent->n},
//                        decltype(parent->t){parent->t},
//                        decltype(parent->nt){parent->nt},
//                        map_.get_x(parent->n),
//                        map_.get_y(parent->n),
//                        current->g,
//                        current->f - current->g,
//                        current->f,
//                        make_goal_state_string(&current->state_[0], nb_goal_crossings),
//                        make_rectangle_state_string(&current->state_[nb_goal_crossings], nb_rect_crossings));
//
//                fmt::print("Found path with cost {}: ", path_cost);
//                for (const auto nt : path)
//                {
//                    fmt::print("({},{}) ", map_.get_x(nt.n), map_.get_y(nt.n));
//                }
//                println("");
//            }
//#endif
//
//            // Check.
//            debug_assert(path_cost <= max_cost);
//            debug_assert(goal_earliest <= current->t && current->t <= goal_latest);
//
//            // Finish.
//            break;
//        }
//
//        // Advance to the next node.
//        ++idx;
//    }
//
//#ifdef DEBUG
//    if (verbose)
//    {
//        println("END DEBUG");
//        println("=========================================================");
//    }
//#endif

    // Return.
    return output;
}

void AStar::set_verbose(const bool on)
{
    verbose = on;
}
#endif

}
