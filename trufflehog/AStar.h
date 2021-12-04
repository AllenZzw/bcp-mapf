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

#ifndef TRUFFLEHOG_ASTAR_H
#define TRUFFLEHOG_ASTAR_H

#include "Includes.h"
#include "Coordinates.h"
#include "Map.h"
#include "LabelPool.h"
#include "ReservationTable.h"
#include "PriorityQueue.h"
#include "Penalties.h"
#include "Heuristic.h"

namespace TruffleHog
{

class AStar
{
    // Label for main low-level search
    struct Label
    {
#ifdef DEBUG
        size_t label_id;
#endif
        Label* parent;
        Cost g;
        Cost f;
        union
        {
#ifdef USE_RESERVATION_TABLE
            struct
            {
                uint64_t nt : 63;
                bool reserved : 1;
            };
            struct
            {
                Location n : 32;
                Timepoint t : 31;
            };
#else
            uint64_t nt;
            struct
            {
                Location n;
                Timepoint t;
            };
#endif
        };
        Int pqueue_index;
        std::byte state_[0];
    };
#ifdef DEBUG
    static_assert(sizeof(Label) == 5*8 + 1*4 + 4);
#else
    static_assert(sizeof(Label) == 4*8 + 1*4 + 4);
#endif

    // Comparison of labels
    struct LabelCompare
    {
#ifdef USE_RESERVATION_TABLE
        ReservationTable reservation_table_;

        LabelCompare(const Int map_size) : reservation_table_(map_size) {}
#endif

        inline bool operator()(const Label* const a, const Label* const b)
        {
            // Prefer smallest f (shorter path) and break ties with smallest reserved
            // status (not reserved) and then largest g (near the end).
#ifdef USE_RESERVATION_TABLE
            return (a->f <  b->f) ||
                   (a->f == b->f && a->reserved <  b->reserved) ||
                   (a->f == b->f && a->reserved == b->reserved && a->g >  b->g) ||
                   (a->f == b->f && a->reserved == b->reserved && a->g == b->g && static_cast<bool>(rand() % 2));
#else
            return (a->f <  b->f) ||
                   (a->f == b->f && a->g >  b->g) ||
                   (a->f == b->f && a->g == b->g && static_cast<bool>(rand() % 2));
#endif
        }
    };

    // Priority queue holding labels
    class AStarPriorityQueue : public PriorityQueue<Label, LabelCompare>
    {
        friend class AStar;

      public:
        // Inherit constructors.
        using PriorityQueue::PriorityQueue;

        // Check.
#ifdef DEBUG
        void check() const
        {
            for (Int pqueue_index = 0; pqueue_index < size_; ++pqueue_index)
            {
                debug_assert(elts_[pqueue_index]->pqueue_index == pqueue_index);
            }
        }
        void check_label(const Label* const label)
        {
            debug_assert(label &&
                         (-1 == label->pqueue_index ||
                          (label->pqueue_index < size_ && elts_[label->pqueue_index] == label)));
        }
#endif

      protected:
        // Modify the handle in the label pointing to its position in the priority queue
        inline void update_pqueue_index(Label* label, const Int pqueue_index)
        {
            label->pqueue_index = pqueue_index;
        }

        // Reprioritise an element up or down
        void decrease_key(Label* label)
        {
            debug_assert(contains(label));
            heapify_up(label->pqueue_index);
        }
        void increase_key(Label* label)
        {
            debug_assert(contains(label));
            heapify_down(label->pqueue_index);
        }

#ifdef DEBUG
        // Check if the priority queue contains label
        inline bool contains(Label* label) const
        {
            const auto index = label->pqueue_index;
            return index < size_ && label == elts_[index];
        }
#endif
    };

  public:
    struct Data
    {
        // Waypoints
        Location start;
        Vector<LocationTimepoint> waypoints;
        Location goal;
        Timepoint earliest_goal_time;
        Timepoint latest_goal_time;

        // Costs
        Cost cost_offset; // dual value for set partition constraint 
        Vector<Timepoint> latest_visit_time; // the latest visited time for locations 
        EdgePenalties edge_penalties; // dual values associated with edges
        FinishTimepointPenalties finish_time_penalties; // dual values associated with wait edge at the goal location 
#ifdef USE_GOAL_CONFLICTS
        GoalPenalties goal_penalties;
#endif

        // Check if any cost is better
        bool can_be_better(const Data& previous_data);
        void smooth(const Data& previous_data, const double SMOOTH_FACTOR); 
    };

  private:
    // ProblemInstance
    const Map& map_;

    // Inputs for a run
    Data data_;

    // Solver data structures
    const Vector<IntCost>* h_node_to_waypoint_;
    Vector<IntCost> h_waypoint_to_goal_;
    Heuristic heuristic_;
    LabelPool label_pool_;
    AStarPriorityQueue open_;
    HashTable<LocationTimepoint, Label*> frontier_;
#ifdef DEBUG
    size_t nb_labels_;
#endif

  public:
    // Constructors
    AStar() = delete;
    AStar(const Map& map);
    AStar(const AStar&) = delete;
    AStar(AStar&&) = delete;
    AStar& operator=(const AStar&) = delete;
    AStar& operator=(AStar&&) = delete;
    ~AStar() = default;

    // Getters
    inline auto max_path_length() const { return heuristic_.max_path_length(); }
#ifdef USE_RESERVATION_TABLE
    auto& reservation_table() { return open_.cmp().reservation_table_; };
#endif
    auto& data() { return data_; }
    const auto& data() const { return data_; }

    // Solve
    inline void compute_h(const Location goal) { heuristic_.get_h(goal); }
    void preprocess_input();
    template<bool is_farkas>
    Pair<Vector<LocationTimepoint>, Cost> solve();
    // template<bool is_farkas>
    // Pair<Vector<LocationTimepoint>, Cost> enumerate();
    // bool before_enumerate(); 

    // Debug
#ifdef DEBUG
    Pair<Vector<LocationTimepoint>, Cost> calculate_cost(const Vector<Edge>& input_path);
    void set_verbose(const bool on = true);
#endif

  private:
    // Check if a label is dominated by an existing label
    AStar::Label* dominated(Label* const new_label);

    // Solve
    void generate_start();
    template<IntCost default_cost>
    void generate_neighbours(Label* const current, const Waypoint w, const Timepoint waypoint_time);
    void generate(Label* const current,
                  const Waypoint w,
                  const Location next_n,
                  const Cost cost,
                  const Timepoint waypoint_time);
    template<IntCost default_cost>
    void generate_neighbours_last_segment(Label* const current);
    void generate_last_segment(Label* const current, const Location next_n, const Cost cost);
    void generate_end(Label* const current);
};

}

#endif
