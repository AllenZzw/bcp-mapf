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

#include "Pricer_TruffleHog.h"
#include "Includes.h"
#include "ProblemData.h"
#include "VariableData.h"
#include "scip/cons_setppc.h"
#include "ConstraintHandler_VertexConflicts.h"
#include "ConstraintHandler_EdgeConflicts.h"
#include "Constraint_VertexBranching.h"
//#include "Constraint_WaitBranching.h"
#include "Constraint_LengthBranching.h"
#include <chrono>
#include <numeric>

#include "trufflehog/ProblemInstance.h"
#include "trufflehog/AStar.h"
#include "lns/inc/LNS.h"

// Pricer properties
#define PRICER_NAME     "trufflehog"
#define PRICER_DESC     "Truffle Hog pricer"
#define PRICER_PRIORITY 0
#define PRICER_DELAY    TRUE    // Only call pricer if all problem variables have non-negative reduced costs

#define EPS (1e-6)
#define STALLED_NB_ROUNDS (4)
#define STALLED_ABSOLUTE_CHANGE (-1)
#define LNS_STEPS (10)
#define SMOOTH_FACTOR (0.5)
#define MAX_SMOOTH_ROUND (3)
#define MAX_HEURISTIC_ROUND (10) 

struct PricingOrder
{
    Robot a;
    bool must_price;
    SCIP_VAR* new_var;
};

// Pricer data
struct SCIP_PricerData
{
    SCIP_CONSHDLR* vertex_branching_conshdlr;           // Constraint handler for vertex branching
//    SCIP_CONSHDLR* wait_branching_conshdlr;           // Constraint handler for wait branching
    SCIP_CONSHDLR* length_branching_conshdlr;           // Constraint handler for length branching
    Robot N;                                            // Number of agents

    SCIP_Real* agent_part_dual;                         // Dual variable values of agent set partition constraints
    SCIP_Real* price_priority;                          // Pricing priority of each agent
    bool* agent_priced;                                 // Indicates if an agent is priced in the current round
    PricingOrder* order;                                // Order of agents to price

#ifdef USE_ASTAR_SOLUTION_CACHING
    Vector<AStar::Data> previous_data;                  // Inputs to the previous run for an agent
#endif
#ifdef USE_LNS
    Vector<SCIP_VAR *> lns_vars;                             // variable index for the solution from the lns solver 
#endif 
#ifdef USE_DUAL_STABILIZATION
    Vector<SCIP_Real> previous_agent_dual;              // previous dual values for partition constraints 
    HashTable<int, SCIP_Real> previous_row_dual;        // previous dual values for LP rows 
    SCIP_Real previous_total_dual;                      // previous total dual values for LP 
#endif
    int heuristic_round; 
    SCIP_Longint last_solved_node;                      // Location number of the last node pricing
    SCIP_Real last_solved_lp_obj[STALLED_NB_ROUNDS];    // LP objective in the last few rounds of pricing
};

// Initialize pricer (called after the problem was transformed)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
static
SCIP_DECL_PRICERINIT(pricerTruffleHogInit)
{
    // Check.
    debug_assert(scip);
    debug_assert(pricer);

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    debug_assert(probdata);

    // Create pricer data.
    SCIP_PricerData* pricerdata;
    SCIP_CALL(SCIPallocBlockMemory(scip, &pricerdata));
    new (pricerdata) SCIP_PricerData;
    pricerdata->N = SCIPprobdataGetN(probdata);
    pricerdata->last_solved_node = -1;
    pricerdata->heuristic_round = MAX_HEURISTIC_ROUND; 

    // Find constraint handler for branching decisions.
    pricerdata->vertex_branching_conshdlr = SCIPfindConshdlr(scip, "vertex_branching");
    release_assert(pricerdata->vertex_branching_conshdlr,
                   "Constraint handler for vertex branching is missing");
//    pricerdata->wait_branching_conshdlr = SCIPfindConshdlr(scip, "wait_branching");
//    release_assert(pricerdata->wait_branching_conshdlr,
//                   "Constraint handler for wait branching is missing");
    pricerdata->length_branching_conshdlr = SCIPfindConshdlr(scip, "length_branching");
    release_assert(pricerdata->length_branching_conshdlr,
                   "Constraint handler for length branching rule is missing");

    // Create array for dual variable values of agent partition constraints.
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &pricerdata->agent_part_dual, pricerdata->N));
    // Overwritten in each run. No need for initialisation.

    // Create array for agent priority in pricing.
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &pricerdata->price_priority, pricerdata->N));
    memset(pricerdata->price_priority, 0, sizeof(SCIP_Real) * pricerdata->N);

    // Create array for priced indicator.
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &pricerdata->agent_priced, pricerdata->N));
    memset(pricerdata->agent_priced, 0, sizeof(bool) * pricerdata->N);

    // Create array for order of agents to price.
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &pricerdata->order, pricerdata->N));
    // Overwritten in each run. No need for initialisation.

    // Create space to store the penalties from the previous failed iteration.
#ifdef USE_ASTAR_SOLUTION_CACHING
    pricerdata->previous_data.resize(pricerdata->N);
#endif

    // Create space to store the variable index from the solution of LNS solver 
#ifdef USE_LNS
    pricerdata->lns_vars.resize(pricerdata->N); 
#endif 

    // Create space to store dual value for stabilization 
#ifdef USE_DUAL_STABILIZATION
    pricerdata->previous_agent_dual.resize(pricerdata->N);              // previous dual values for partition constraints 
    pricerdata->previous_total_dual = 0.0; 
#endif

    // Set pointer to pricer data.
    SCIPpricerSetData(pricer, pricerdata);
    SCIPprobdataSetPricerData(probdata, pricerdata);

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Free pricer
static
SCIP_DECL_PRICERFREE(pricerTruffleHogFree)
{
    // Check.
    debug_assert(scip);
    debug_assert(pricer);

    // Get pricer data.
    auto pricerdata = SCIPpricerGetData(pricer);
    debug_assert(pricerdata);

    // Deallocate.
    SCIPfreeBlockMemoryArray(scip, &pricerdata->agent_part_dual, pricerdata->N);
    SCIPfreeBlockMemoryArray(scip, &pricerdata->price_priority, pricerdata->N);
    SCIPfreeBlockMemoryArray(scip, &pricerdata->agent_priced, pricerdata->N);
    SCIPfreeBlockMemoryArray(scip, &pricerdata->order, pricerdata->N);
    pricerdata->~SCIP_PricerData();
    SCIPfreeBlockMemory(scip, &pricerdata);

    // Done.
    return SCIP_OKAY;
}

enum class MasterProblemStatus
{
    Infeasible = 0,
    Fractional = 1,
    Integral = 2
};

// Compute ordering of agents to price
MasterProblemStatus calculate_agents_order(
    SCIP* scip,                    // SCIP
    SCIP_PROBDATA* probdata,       // Problem data
    SCIP_PricerData* pricerdata    // Pricer data
)
{
    // Get problem data.
    const auto N = SCIPprobdataGetN(probdata);

    // Get variables.
    const auto& dummy_vars = SCIPprobdataGetDummyVars(probdata);
    const auto& agent_vars = SCIPprobdataGetRobotVars(probdata);

    // Calculate the order of the agents.
    MasterProblemStatus master_lp_status = MasterProblemStatus::Integral;
    auto order = pricerdata->order;
    for (Robot a = 0; a < N; ++a)
    {
        // Must price an agent if it is using an artificial variable.
        bool must_price_agent = false;
        if (SCIPisPositive(scip, SCIPgetSolVal(scip, nullptr, dummy_vars[a])))
        {
            master_lp_status = MasterProblemStatus::Infeasible;
            must_price_agent = true;
        }

        // Must price an agent if it is fractional.
        if (!must_price_agent)
        {
            for (auto var : agent_vars[a])
            {
                // Get the variable value.
                debug_assert(var);
                const auto var_val = SCIPgetSolVal(scip, nullptr, var);

                // Set.
                if (!SCIPisIntegral(scip, var_val))
                {
                    master_lp_status = std::min(master_lp_status, MasterProblemStatus::Fractional);
                    // must_price_agent = true;
                    break;
                }
            }
        }

        // Store.
        order[a] = {a, must_price_agent, nullptr};
    }

    // Price all agents if the master problem solution is integral.
    if (master_lp_status == MasterProblemStatus::Integral)
    {
        debugln("must price all agent when the master problem solution is integral.");
        for (Robot a = 0; a < N; ++a)
        {
            order[a].must_price = true;
        }
    }

    // Sort.
    {
        auto price_priority = pricerdata->price_priority;
        std::sort(order,
                  order + N,
                  [price_priority](const PricingOrder& a, const PricingOrder& b)
                  {
                      return (a.must_price >  b.must_price) ||
                             (a.must_price == b.must_price && price_priority[a.a] > price_priority[b.a]);
                  });
    }

    // Reset.
    memset(pricerdata->agent_priced, 0, sizeof(bool) * pricerdata->N);

    // Done.
    return master_lp_status;
}

static bool run_lns_pricer(
    SCIP* scip, 
    SCIP_PRICER* pricer      // Pricer
)
{
    // Get problem data.
    SCIP_ProbData* probdata = SCIPgetProbData(scip);
    const auto& map = SCIPprobdataGetMap(probdata);
    auto& lns = SCIPprobdataGetLNS(probdata);
    auto& lns_instance = SCIPprobdataGetLNSInstance(probdata);
    auto& agents_vars = SCIPprobdataGetRobotVars(probdata); 
    auto pricerdata = SCIPpricerGetData(pricer);
    auto N = SCIPprobdataGetN(probdata); 

    // run LNS to search for new columns 
    bool succ = false; 
    if (lns.initial_sum_of_costs == -1) 
    {
        lns.sum_of_distances = 0; 
        for (const auto & agent : lns.agents)
            lns.sum_of_distances += agent.path_planner.my_heuristic[agent.path_planner.start_location];
        
        const auto start_time = std::chrono::high_resolution_clock::now();
        while (!succ && ((fsec)(Time::now() - start_time)).count() < lns.time_limit)
            succ = lns.getInitialSolution();
    }
    else 
    {
        for (int i = 0; i < LNS_STEPS && !succ; i++) 
            succ = lns.runOneStepSearch();
    }
    
    if (succ)
    {
        debugln("Run LNS solver get solution with obj {}", lns.sum_of_costs ); 
        auto& neighbor = lns.getNeighbor();  
        for (auto a : neighbor.agents) 
        {
            Vector<Edge> tmp_path;
            auto& path = lns.getPath(a); 
            Agent agent = lns.agents[a]; 
            auto [y, x] = lns_instance.getCoordinate(agent.path[0].location); 
            for (int i = 1; i != agent.path.size(); i++)
            {
                const auto& state = agent.path[i];
                Direction d; 
                auto [next_y, next_x] = lns_instance.getCoordinate(agent.path[i].location); 
                if (next_y == y - 1)
                    tmp_path.push_back(Edge(map.get_id(x+1,y+1), NORTH));
                else if (next_y == y + 1)
                    tmp_path.push_back(Edge(map.get_id(x+1,y+1), SOUTH));
                else if (next_x == x + 1)
                    tmp_path.push_back(Edge(map.get_id(x+1,y+1), EAST));
                else if (next_x == x - 1)
                    tmp_path.push_back(Edge(map.get_id(x+1,y+1), WEST));
                else 
                    tmp_path.push_back(Edge(map.get_id(x+1,y+1), WAIT));
                x = next_x;
                y = next_y;
            }
            tmp_path.push_back(Edge(map.get_id(x+1,y+1), Direction::INVALID));

            // checking whether the path exists or not 
            bool exists = false; 
            for (auto var : agents_vars.at(a))
            {
                debug_assert(var);
                auto vardata = SCIPvarGetData(var);
                const auto existing_path_length = SCIPvardataGetPathLength(vardata);
                const auto existing_path = SCIPvardataGetPath(vardata);
                if (std::equal(tmp_path.data(), tmp_path.data() + tmp_path.size(), existing_path, existing_path + existing_path_length))
                {
                    exists = true; 
                    pricerdata->lns_vars[a] = var; 
                    break; 
                }
            }

            if (!exists)
            {
                // Add column.
                SCIP_VAR* var = nullptr;
                SCIP_CALL(SCIPprobdataAddPricedVar(scip, probdata, a, tmp_path.size(), tmp_path.data(), &var));
                debug_assert(var); 
                auto vardata = SCIPvarGetData(var);
                pricerdata->lns_vars[a] = var; 
            }
        }

        SCIP_SOL * newsol; 
        SCIP_Bool success;
        SCIP_CALL(SCIPcreateSol(scip, &newsol, nullptr)); 
        for (int a = 0; a < N; a++)
        {
            for (auto var : agents_vars.at(a))
            {
                auto vardata = SCIPvarGetData(var);
                if ( SCIPvarGetIndex(pricerdata->lns_vars[a]) == SCIPvarGetIndex(var) )
                    SCIPsetSolVal(scip, newsol, var, 1.0);
                else
                    SCIPsetSolVal(scip, newsol, var, 0.0);
            }
        }
        // print_used_paths(scip, newsol); 
        SCIP_CALL(SCIPtrySol(scip, newsol, FALSE, FALSE, FALSE, FALSE, FALSE, &success));
        debug_assert(success); 
        return true; 
    }
    else
        return false; 
}

// a debug pricer to run the pricing problem for all agents at each pricing round 
static 
SCIP_RETCODE run_full_pricer(
    SCIP* scip,               // SCIP
    SCIP_PRICER* pricer,      // Pricer
    SCIP_RESULT* result,      // Output result
    SCIP_Bool* stopearly,     // Output flag to indicate early branching is required
    SCIP_Real* lower_bound    // Output lower bound
)
{
    // No Farkas pricing.
    // constexpr bool is_farkas = false;

    // Check.
    debug_assert(scip);
    debug_assert(pricer);

    // Get pricer data.
    auto pricerdata = SCIPpricerGetData(pricer);
    debug_assert(pricerdata);

    // Print.
    debugln("Starting pricer for feasible master problem at node {}, depth {}, node LB {}, LP obj {}:",
            SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
            SCIPgetDepth(scip),
            SCIPgetNodeLowerbound(scip, SCIPgetCurrentNode(scip)),
            SCIPgetLPObjval(scip));

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& map = SCIPprobdataGetMap(probdata);
    const auto& agents = SCIPprobdataGetRobotsData(probdata);
    auto& lns = SCIPprobdataGetLNS(probdata);

#ifdef USE_LNS
    if (SCIPnodeGetNumber(SCIPgetCurrentNode(scip)) == 1 && run_lns_pricer(scip, pricer))
    {
        *lower_bound = std::max(lns.sum_of_distances, lns.sum_of_costs_lowerbound); 
        *result = SCIP_SUCCESS;
        return SCIP_OKAY;
    }
#endif

    // Create order of agents to solve.
    auto order = pricerdata->order;
    const auto master_lp_status = calculate_agents_order(scip, probdata, pricerdata);
    for (Robot a = 0; a < N; ++a)
    {
        pricerdata->price_priority[a] /= PRICE_PRIORITY_DECAY_FACTOR;
    }

    // Early branching if LP is stalled.
    if (master_lp_status == MasterProblemStatus::Fractional || SCIPgetLPObjval(scip) != SCIPgetPrimalbound(scip) )
    // if (master_lp_status == MasterProblemStatus::Fractional )
    {
        const auto current_node = SCIPnodeGetNumber(SCIPgetCurrentNode(scip));
        if (pricerdata->last_solved_node != current_node)
        {
            constexpr auto nan = std::numeric_limits<SCIP_Real>::quiet_NaN();
            std::fill(pricerdata->last_solved_lp_obj, pricerdata->last_solved_lp_obj + STALLED_NB_ROUNDS, nan);
            // pricerdata->last_solved_node = current_node;
        }
        else
        {
            std::memmove(pricerdata->last_solved_lp_obj,
                            pricerdata->last_solved_lp_obj + 1,
                            sizeof(SCIP_Real) * (STALLED_NB_ROUNDS - 1));
            pricerdata->last_solved_lp_obj[STALLED_NB_ROUNDS - 1] = SCIPgetLPObjval(scip);
            
            debugln("");
            debugln("   LP history: {}", fmt::join(pricerdata->last_solved_lp_obj, pricerdata->last_solved_lp_obj + STALLED_NB_ROUNDS, " "));

            bool stalled = true;
            for (Int idx = 0; idx < STALLED_NB_ROUNDS - 1; ++idx)
            {
                const auto change = pricerdata->last_solved_lp_obj[idx + 1] - pricerdata->last_solved_lp_obj[idx];
                debugln("   LP absolute change {}", change);
                if (!(change <= 0 && change >= STALLED_ABSOLUTE_CHANGE)) // Stalled if change is positive or less than some amount
                {
                    stalled = false;
                    break;
                }
            }
            if (stalled)
            {
                debugln("   LP stalled - skip pricing");
                *stopearly = true;
                return SCIP_OKAY;
            }
            else
                debugln("   LP not stalled - start pricing");
        }
    }

    // Get variables.
    auto& vars = SCIPprobdataGetVars(probdata);

    // Get constraints.
    const auto& agent_part = SCIPprobdataGetRobotPartConss(probdata);
    const auto& vertex_conflicts_conss = vertex_conflicts_get_constraints(probdata);
    const auto& edge_conflicts_conss = edge_conflicts_get_constraints(probdata);
    const auto& agent_goal_vertex_conflicts = SCIPprobdataGetRobotGoalVertexConflicts(probdata);
#ifdef USE_WAITEDGE_CONFLICTS
    const auto& agent_goal_edge_conflicts = SCIPprobdataGetRobotGoalEdgeConflicts(probdata);
#endif

    // Get cuts.
    const auto& agent_robust_cuts = SCIPprobdataGetRobotRobustCuts(probdata);
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
    const auto& rectangle_clique_conflicts_conss = rectangle_clique_conflicts_get_constraints(probdata);
#endif
#ifdef USE_GOAL_CONFLICTS
    const auto& goal_agent_goal_conflicts = SCIPprobdataGetGoalRobotGoalConflicts(probdata);
    const auto& crossing_agent_goal_conflicts = SCIPprobdataGetCrossingRobotGoalConflicts(probdata);
#endif
#ifdef USE_PATH_LENGTH_NOGOODS
    const auto& path_length_nogoods = SCIPprobdataGetPathLengthNogoods(probdata);
#endif

    // Get constraints for branching decisions.
    const auto n_vertex_branching_conss = SCIPconshdlrGetNConss(pricerdata->vertex_branching_conshdlr);
    auto vertex_branching_conss = SCIPconshdlrGetConss(pricerdata->vertex_branching_conshdlr);
    debug_assert(n_vertex_branching_conss == 0 || vertex_branching_conss);
    const auto n_length_branching_conss = SCIPconshdlrGetNConss(pricerdata->length_branching_conshdlr);
    auto length_branching_conss = SCIPconshdlrGetConss(pricerdata->length_branching_conshdlr);
    debug_assert(n_length_branching_conss == 0 || length_branching_conss);

    // Get the low-level solver.
    auto& astar = SCIPprobdataGetAStar(probdata);

    // Get data from the low-level solver.
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
    ] = astar.data();

    // Find the makespan.
    Timepoint makespan = 0;
    for (auto var : vars)
    {
        // Get the path length.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto path_length = SCIPvardataGetPathLength(vardata);

        // Store the length of the longest path.
        if (path_length > makespan)
        {
            makespan = path_length;
        }
    }

//#ifdef PRINT_DEBUG
//    print_agent_part_dual(scip, false);
//    print_vertex_conflicts_dual(scip, false);
//    print_edge_conflicts_dual(scip, false);
//    print_two_agent_robust_cuts_dual(scip, false);
// #ifdef USE_GOAL_CONFLICTS
//    print_goal_conflicts_dual(scip, false);
// #endif
// #ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
//    print_rectangle_clique_conflicts_dual(scip, false);
// #endif
//#endif

    // Price each agent.
    Int nb_new_cols = 0;
    SCIP_Real primal_lb, dual_ub, total_dual; 
    Vector<SCIP_Real> current_agent_dual(N);              // current dual values for partition constraints 
    HashTable<int, SCIP_Real> current_row_dual;        // current dual values for LP rows 
    bool misprice = true;
    int smooth_round = pricerdata->last_solved_node != -1? MAX_SMOOTH_ROUND: 1; 
    auto agent_priced = pricerdata->agent_priced;
    do {
        if (smooth_round == MAX_SMOOTH_ROUND || smooth_round == 1)
            total_dual = SCIPgetLPObjval(scip); 
        if (smooth_round > 1)
        {
            total_dual = SMOOTH_FACTOR * pricerdata->previous_total_dual + (1.0 - SMOOTH_FACTOR) * SCIPgetLPObjval(scip); 
            debugln("       Total dual value at round {} is {}, previous total dual value: {}, smoothed dual value: {}", smooth_round, SCIPgetLPObjval(scip), pricerdata->previous_total_dual, total_dual); 
        }

        // Set up dual value for agent partition constraint 
        for (Int a = 0; a < N; ++a)
        {
            // Get the constraint.
            auto cons = agent_part[a];
            debug_assert(cons);

            // Check that the constraint is not (locally) disabled/redundant.
            debug_assert(SCIPconsIsEnabled(cons));

            // Check that no variable is fixed to one.
            debug_assert(SCIPgetNFixedonesSetppc(scip, cons) == 0);

            // Store dual value.
            if (smooth_round == MAX_SMOOTH_ROUND || smooth_round == 1)
                current_agent_dual[a] = SCIPgetDualsolSetppc(scip, cons);
            if (smooth_round > 1)
            {
                current_agent_dual[a] = SMOOTH_FACTOR * pricerdata->previous_agent_dual[a] + (1.0 - SMOOTH_FACTOR) * current_agent_dual[a]; 
                debugln("       Dual value for agent {} at round {} is {}, previous dual value: {}, smoothed dual value: {}", a, smooth_round,SCIPgetDualsolSetppc(scip, cons), pricerdata->previous_agent_dual[a], current_agent_dual[a]); 
            }
        }

        // Set up dual for LP rows 
        current_row_dual.clear();
        SCIP_ROW ** rows = SCIPgetLPRows(scip); 
        int row_nb = SCIPgetNLPRows(scip);
        for (Int i = 0; i < row_nb; i++)
        {
            // Get the row 
            SCIP_ROW * row = rows[i]; 
            debug_assert(row);

            // Store dual value if it is in LP 
            if (SCIProwIsInLP(row))
            {
                if (smooth_round == MAX_SMOOTH_ROUND || smooth_round == 1)
                    current_row_dual[SCIProwGetIndex(row)] = SCIProwGetDualsol(row);
                if (smooth_round > 1)
                {
                    current_row_dual[SCIProwGetIndex(row)] = (1.0 - SMOOTH_FACTOR) * current_row_dual[SCIProwGetIndex(row)];
                    if (pricerdata->previous_row_dual.contains(SCIProwGetIndex(row))) 
                    {
                        current_row_dual[SCIProwGetIndex(row)] += SMOOTH_FACTOR * pricerdata->previous_row_dual[SCIProwGetIndex(row)]; 
                        debugln("       Dual value at round {} for row {} is {}, previous dual value: {}, smoothed dual value: {}", smooth_round, SCIProwGetIndex(row), SCIProwGetDualsol(row), pricerdata->previous_row_dual[SCIProwGetIndex(row)], current_row_dual[SCIProwGetIndex(row)]); 
                    }
                }
            }
        }

        // Make edge penalties for all agents.
        EdgePenalties global_edge_penalties;
        // Input dual values for vertex conflicts.
        for (const auto& [nt, vertex_conflict] : vertex_conflicts_conss)
        {
            const auto& [row] = vertex_conflict;
            const auto dual = current_row_dual[SCIProwGetIndex(row)];
            // const auto dual = SCIProwGetDualsol(row);
            debug_assert(SCIPisFeasLE(scip, dual, 0.0));
            if (SCIPisFeasLT(scip, dual, 0.0))
            {
                // Add the dual variable value to the edges leading into the vertex.
                const auto t = nt.t - 1;
                {
                    const auto n = map.get_south(nt.n);
                    auto& penalties = global_edge_penalties.get_edge_penalties(n, t);
                    penalties.north -= dual;
                }
                {
                    const auto n = map.get_north(nt.n);
                    auto& penalties = global_edge_penalties.get_edge_penalties(n, t);
                    penalties.south -= dual;
                }
                {
                    const auto n = map.get_west(nt.n);
                    auto& penalties = global_edge_penalties.get_edge_penalties(n, t);
                    penalties.east -= dual;
                }
                {
                    const auto n = map.get_east(nt.n);
                    auto& penalties = global_edge_penalties.get_edge_penalties(n, t);
                    penalties.west -= dual;
                }
                {
                    const auto n = map.get_wait(nt.n);
                    auto& penalties = global_edge_penalties.get_edge_penalties(n, t);
                    penalties.wait -= dual;
                }
            }
        }

        // Input dual values for edge conflicts.
        for (const auto& [et, edge_conflict] : edge_conflicts_conss)
        {
            const auto& [row, edges, t] = edge_conflict;
            const auto dual = current_row_dual[SCIProwGetIndex(row)];
            // const auto dual = SCIProwGetDualsol(row);
            debug_assert(SCIPisFeasLE(scip, dual, 0.0));
            if (SCIPisFeasLT(scip, dual, 0.0))
            {
                // Add the dual variable value to the edges.
                for (const auto e : edges)
                {
                    auto& penalties = global_edge_penalties.get_edge_penalties(e.n, t);
                    penalties.d[e.d] -= dual;
                }
            }
        }
    
        primal_lb = SCIPgetLPObjval(scip); 
        dual_ub = total_dual; 
        // for (Int order_idx = 0; order_idx < N && (!nb_new_cols < 1 || order[order_idx].must_price) && !SCIPisStopped(scip); ++order_idx)
        for (Int order_idx = 0; order_idx < N && !SCIPisStopped(scip); ++order_idx)
        {
            // Create output.
            Vector<Edge> path;
            Vector<LocationTimepoint> path_vertices;
            SCIP_Real path_cost;

            // Set up start and end points.
            const auto a = order[order_idx].a;
            start = agents[a].start;
            goal = agents[a].goal;

            // Input the agent partition dual.
            {
                // Store dual value.
                const auto dual = current_agent_dual[a]; 
                debug_assert(SCIPisGE(scip, dual, 0.0));
                cost_offset = -dual;
            }

            // Modify edge costs for two-agent robust cuts.
            edge_penalties = global_edge_penalties;
            finish_time_penalties.clear();
    #ifdef USE_GOAL_CONFLICTS
            goal_penalties.clear();
    #endif
            for (const auto& [row, ets_begin, ets_end] : agent_robust_cuts[a])
            {
                // const auto dual = SCIProwGetDualsol(row);
                const auto dual = current_row_dual[SCIProwGetIndex(row)];
                debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                if (SCIPisFeasLT(scip, dual, 0.0))
                {
                    for (auto it = ets_begin; it != ets_end; ++it)
                    {
                        // Incur the penalty on the edge.
                        const auto n = it->n;
                        const auto d = it->d;
                        const auto t = it->t;
                        auto& penalties = edge_penalties.get_edge_penalties(n, t);
                        penalties.d[d] -= dual;

                        // If a wait edge in a two-agent robust cut corresponds to waiting at the goal, incur the
                        // penalty for staying at the goal because the low-level solver doesn't traverse this edge.
                        if (n == goal && d == Direction::WAIT)
                        {
                            const auto conflict_time = it->t;
                            finish_time_penalties.add(conflict_time, -dual);
                        }
                    }
                }
            }

            // If the node of a vertex conflict is the goal, incur a penalty for waiting (indefinitely) at the goal
            // after the agent has completed its path.
            for (const auto& [t, row] : agent_goal_vertex_conflicts[a])
            {
                // const auto dual = SCIProwGetDualsol(row);
                const auto dual = current_row_dual[SCIProwGetIndex(row)];
                debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                if (SCIPisFeasLT(scip, dual, 0.0))
                {
                    // Add the penalty if the agent finishes before time t. The penalty at time t is accounted for in the
                    // global edge penalties.
                    finish_time_penalties.add(t - 1, -dual);
                }
            }

            // If the wait edge in a wait edge conflict is at the goal, incur a penalty for staying at the goal.
    #ifdef USE_WAITEDGE_CONFLICTS
            for (const auto& [t, row] : agent_goal_edge_conflicts[a])
            {
                // const auto dual = SCIProwGetDualsol(row);
                const auto dual = current_row_dual[SCIProwGetIndex(row)];
                debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                if (SCIPisFeasLT(scip, dual, 0.0))
                {
                    // If an agent finishes at time t, it does not traverse the edge at time t to t+1. So the agent is 
                    // penalised here.
                    finish_time_penalties.add(t, -dual);
                }
            }
    #endif

            // Add goal crossings or finish time penalties for goal conflicts. If agent a1 finishes at or before time t,
            // incur the penalty. If agent a2 crosses the goal of agent a1 at or after time t, incur the penalty.
    #ifdef USE_GOAL_CONFLICTS
            for (const auto& [t, row] : goal_agent_goal_conflicts[a])
            {
                // const auto dual = SCIProwGetDualsol(row);
                const auto dual = current_row_dual[SCIProwGetIndex(row)];
                debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                if (SCIPisFeasLT(scip, dual, 0.0))
                {
                    finish_time_penalties.add(t, -dual);
                }
            }
            for (const auto& [nt, row] : crossing_agent_goal_conflicts[a])
            {
                // const auto dual = SCIProwGetDualsol(row);
                const auto dual = current_row_dual[SCIProwGetIndex(row)];
                debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                if (SCIPisFeasLT(scip, dual, 0.0))
                {
                    goal_penalties.add(nt, -dual);
                }
            }
    #endif

            // Modify edge costs for path length nogoods. If agent a finishes at or before time t, incur the penalty.
    #ifdef USE_PATH_LENGTH_NOGOODS
            for (const auto& [row, latest_finish_times] : path_length_nogoods)
                for (const auto& [nogood_a, t] : latest_finish_times)
                    if (a == nogood_a)
                    {
                        // const auto dual = SCIProwGetDualsol(row);
                        const auto dual = current_row_dual[SCIProwGetIndex(row)];
                        debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                        if (SCIPisFeasLT(scip, dual, 0.0))
                        {
                            finish_time_penalties.add(t, -dual);
                        }
                    }
    #endif

            // Modify edge costs for vertex branching decisions.
            waypoints.clear();
            for (Int c = 0; c < n_vertex_branching_conss; ++c)
            {
                // Get the constraint.
                auto cons = vertex_branching_conss[c];
                debug_assert(cons);

                // Ignore constraints that are not active since these are not on the current
                // active path of the search tree.
                if (!SCIPconsIsActive(cons))
                    continue;

                // Enforce the decision.
                const auto branch_a = SCIPgetVertexBranchingRobot(cons);
                const auto dir = SCIPgetVertexBranchingDirection(cons);
                const auto nt = SCIPgetVertexBranchingLocationTimepoint(cons);
                if ((a == branch_a && dir == VertexBranchDirection::Forbid) ||
                    (a != branch_a && dir == VertexBranchDirection::Use))
                {
                    // Don't use the vertex.
                    const auto prev_time = nt.t - 1;
                    {
                        const auto n = map.get_south(nt.n);
                        auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                        penalties.north = std::numeric_limits<Cost>::infinity();
                    }
                    {
                        const auto n = map.get_north(nt.n);
                        auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                        penalties.south = std::numeric_limits<Cost>::infinity();
                    }
                    {
                        const auto n = map.get_west(nt.n);
                        auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                        penalties.east = std::numeric_limits<Cost>::infinity();
                    }
                    {
                        const auto n = map.get_east(nt.n);
                        auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                        penalties.west = std::numeric_limits<Cost>::infinity();
                    }
                    {
                        const auto n = map.get_wait(nt.n);
                        auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                        penalties.wait = std::numeric_limits<Cost>::infinity();
                    }
                }
                else if (a == branch_a)
                {
                    debug_assert(dir == VertexBranchDirection::Use);

                    // Store a waypoint to enforce use of the vertex.
                    waypoints.push_back(nt);
                }
            }

            // Sort waypoints by time.
            std::sort(waypoints.begin(), waypoints.end(), [](const auto& a, const auto& b)
            {
                return a.t < b.t;
            });

            // Modify edge costs for length branching decisions.
            debug_assert(astar.max_path_length() >= 1);
            earliest_goal_time = 0;
            latest_goal_time = astar.max_path_length() - 1;
            latest_visit_time = map.latest_visit_time();
            for (Int c = 0; c < n_length_branching_conss; ++c)
            {
                // Get the constraint.
                auto cons = length_branching_conss[c];
                debug_assert(cons);

                // Ignore constraints that are not active since these are not on the current active path of the search tree.
                if (!SCIPconsIsActive(cons))
                    continue;

                // Enforce the decision if the same agent. Disable crossing if different agent.
                const auto branch_a = SCIPgetLengthBranchingRobot(cons);
                const auto dir = SCIPgetLengthBranchingDirection(cons);
                const auto nt = SCIPgetLengthBranchingLocationTimepoint(cons);
                if (a == branch_a)
                {
                    if (dir == LengthBranchDirection::LEq)
                    {
                        latest_goal_time = std::min(latest_goal_time, nt.t);
                    }
                    else
                    {
                        earliest_goal_time = std::max(earliest_goal_time, nt.t);
                    }
                }
                else if (dir == LengthBranchDirection::LEq)
                {
                    // Block the vertex at time t and onwards 
                    // since the agent will stay at location nt.n after time t
                    latest_visit_time[nt.n] = nt.t - 1;
                }
            }
            debug_assert(waypoints.empty() || latest_goal_time >= waypoints.back().t);

            // Preprocess input data.
            astar.preprocess_input();

            // Skip running A* if the penalties in the last iteration of this agent have stayed the same or worsened.
    #ifdef USE_ASTAR_SOLUTION_CACHING
            if (!astar.data().can_be_better(pricerdata->previous_data[a]))
            {
                goto FINISHED_PRICING_AGENT;
            }
    #endif

            // Solve.
            std::tie(path_vertices, path_cost) = astar.solve<false>();
            if (!path_vertices.empty())
            {
                primal_lb += path_cost; 
                dual_ub += path_cost; 
                // Get the solution.
                for (auto it = path_vertices.begin(); it != path_vertices.end(); ++it)
                {
                    const auto d = it != path_vertices.end() - 1 ?
                                map.get_direction(it->n, (it + 1)->n) :
                                Direction::INVALID;
                    path.push_back(Edge{it->n, d});
                }

                // Add a column only if the path has negative reduced cost.
                if (SCIPisSumLT(scip, path_cost, 0.0))
                {
                    // Print.
                    debugln("       Found path for agent {} with length {}, reduced cost {:.6f} ({})",
                            a, 
                            path.size(),
                            path_cost,
                            format_path(probdata, path.size(), path.data()));
                    
                    // Add column.
                    SCIP_VAR* var = nullptr;
                    SCIP_CALL(SCIPprobdataAddPricedVar(scip, probdata, a, path.size(), path.data(), &var));
                    debug_assert(var);
                    misprice = false; 
                    order[order_idx].new_var = var;
                    pricerdata->price_priority[a]++;
                    nb_new_cols++;

    #ifdef USE_RESERVATION_TABLE
                    // Update reservation table.
                    {
                        Location n;
                        Timepoint t = 0;
                        for (; t < static_cast<Timepoint>(path.size()); ++t)
                        {
                            n = path[t].n;
                            restab.reserve(LocationTimepoint{n, t});
                        }
                        for (; t < makespan; ++t)
                        {
                            restab.reserve(LocationTimepoint{n, t});
                        }
                    }
    #endif

                    // Advance to the next agent.
                    goto FINISHED_PRICING_AGENT;
                }
            }
            
            // Store the penalties of the run.
    #ifdef USE_ASTAR_SOLUTION_CACHING
            pricerdata->previous_data[a] = astar.data();
    #endif

            // End of this agent.
            FINISHED_PRICING_AGENT:
            agent_priced[a] = true;

            // End timer.
    #ifdef PRINT_DEBUG
            const auto end_time = std::chrono::high_resolution_clock::now();
            const auto duration = std::chrono::duration<double>(end_time - start_time).count();
            debugln("    Done in {:.4f} seconds", duration);
    #endif
        }
        smooth_round--; 
#ifdef USE_DUAL_STABILIZATION
        if (dual_ub >= SCIPgetDualbound(scip))
        {
            pricerdata->previous_total_dual = total_dual; 
            pricerdata->previous_agent_dual = current_agent_dual; 
            pricerdata->previous_row_dual = current_row_dual; 
        }
        if (misprice)
        {
            debugln("   lower bound is set to {}", dual_ub); 
            *lower_bound = dual_ub; 
        }
#endif  
    } while (smooth_round > 0 && misprice); 
    
    // Print.
    debugln("Added {} new columns", nb_new_cols);

    // Finish.
    if (!SCIPisStopped(scip))
    {
        pricerdata->last_solved_node = SCIPnodeGetNumber(SCIPgetCurrentNode(scip));
        if (pricerdata->heuristic_round == 0)
            pricerdata->heuristic_round = MAX_HEURISTIC_ROUND; 
        else 
            pricerdata->heuristic_round--; 
        // Compute lower bound.
        bool all_agents_priced = true;
        for (Robot a = 0; a < N; ++a)
            if (!agent_priced[a])
            {
                all_agents_priced = false;
                break;
            }
        if (all_agents_priced)
        {
            // *lower_bound = primal_lb; 
            *lower_bound = dual_ub; 
            debugln("   All agent priced at node {}, smooth round: {}, primal LP obj: {}, primal LB: {}, total dual: {}, dual UB: {}, *lower_bound: {}, Added {} columns", 
                        pricerdata->last_solved_node, smooth_round, SCIPgetLPObjval(scip), primal_lb, total_dual, dual_ub, *lower_bound, nb_new_cols); 
        }

        // Mark as completed.
        *result = SCIP_SUCCESS;
    }
    return SCIP_OKAY;
}

static
SCIP_RETCODE run_trufflehog_pricer(
    SCIP* scip,               // SCIP
    SCIP_PRICER* pricer,      // Pricer
    SCIP_RESULT* result,      // Output result
    SCIP_Bool* stopearly,     // Output flag to indicate early branching is required
    SCIP_Real* lower_bound    // Output lower bound
)
{
    // No Farkas pricing.
    constexpr bool is_farkas = false;

    // Check.
    debug_assert(scip);
    debug_assert(pricer);

    // Get pricer data.
    auto pricerdata = SCIPpricerGetData(pricer);
    debug_assert(pricerdata);

    // Print.
    if constexpr (!is_farkas)
    {
        debugln("Starting pricer for feasible master problem at node {}, depth {}, node LB {}, LP obj {}:",
                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
                SCIPgetDepth(scip),
                SCIPgetNodeLowerbound(scip, SCIPgetCurrentNode(scip)),
                SCIPgetLPObjval(scip));
    }
    else
    {
        debugln("Starting pricer for infeasible master problem at node {}, depth {}:",
                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
                SCIPgetDepth(scip));
    }

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& map = SCIPprobdataGetMap(probdata);
    const auto& agents = SCIPprobdataGetRobotsData(probdata);

    // Create order of agents to solve.
    auto order = pricerdata->order;
    const auto master_lp_status = calculate_agents_order(scip, probdata, pricerdata);
    for (Robot a = 0; a < N; ++a)
    {
        pricerdata->price_priority[a] /= PRICE_PRIORITY_DECAY_FACTOR;
    }

    // Early branching if LP is stalled.
    if constexpr (!is_farkas)
    {
        if (master_lp_status == MasterProblemStatus::Fractional)
        {
            const auto current_node = SCIPnodeGetNumber(SCIPgetCurrentNode(scip));
            if (pricerdata->last_solved_node != current_node)
            {
                constexpr auto nan = std::numeric_limits<SCIP_Real>::quiet_NaN();
                std::fill(pricerdata->last_solved_lp_obj, pricerdata->last_solved_lp_obj + STALLED_NB_ROUNDS, nan);
                pricerdata->last_solved_node = current_node;
            }
            else
            {
                std::memmove(pricerdata->last_solved_lp_obj,
                             pricerdata->last_solved_lp_obj + 1,
                             sizeof(SCIP_Real) * (STALLED_NB_ROUNDS - 1));
                pricerdata->last_solved_lp_obj[STALLED_NB_ROUNDS - 1] = SCIPgetLPObjval(scip);

                debugln("");
                debugln("   LP history: {}", fmt::join(pricerdata->last_solved_lp_obj, pricerdata->last_solved_lp_obj + STALLED_NB_ROUNDS, " "));

                bool stalled = true;
                for (Int idx = 0; idx < STALLED_NB_ROUNDS - 1; ++idx)
                {
                    const auto change = pricerdata->last_solved_lp_obj[idx + 1] - pricerdata->last_solved_lp_obj[idx];
                    debugln("   LP absolute change {}", change);
                    if (!(change <= 0 && change >= STALLED_ABSOLUTE_CHANGE)) // Stalled if change is positive or less than some amount
                    {
                        stalled = false;
                        break;
                    }
                }
                if (stalled)
                {
                    debugln("   LP stalled - skip pricing");
                    *stopearly = true;
                    return SCIP_OKAY;
                }
                else
                {
                    debugln("   LP not stalled - start pricing");
                }
            }
        }
    }

    // Get variables.
    auto& vars = SCIPprobdataGetVars(probdata);

    // Get constraints.
    const auto& agent_part = SCIPprobdataGetRobotPartConss(probdata);
    const auto& vertex_conflicts_conss = vertex_conflicts_get_constraints(probdata);
    const auto& edge_conflicts_conss = edge_conflicts_get_constraints(probdata);
    const auto& agent_goal_vertex_conflicts = SCIPprobdataGetRobotGoalVertexConflicts(probdata);
#ifdef USE_WAITEDGE_CONFLICTS
    const auto& agent_goal_edge_conflicts = SCIPprobdataGetRobotGoalEdgeConflicts(probdata);
#endif

    // Get cuts.
    const auto& agent_robust_cuts = SCIPprobdataGetRobotRobustCuts(probdata);
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
    const auto& rectangle_clique_conflicts_conss = rectangle_clique_conflicts_get_constraints(probdata);
#endif
#ifdef USE_GOAL_CONFLICTS
    const auto& goal_agent_goal_conflicts = SCIPprobdataGetGoalRobotGoalConflicts(probdata);
    const auto& crossing_agent_goal_conflicts = SCIPprobdataGetCrossingRobotGoalConflicts(probdata);
#endif
#ifdef USE_PATH_LENGTH_NOGOODS
    const auto& path_length_nogoods = SCIPprobdataGetPathLengthNogoods(probdata);
#endif

    // Get constraints for branching decisions.
    const auto n_vertex_branching_conss = SCIPconshdlrGetNConss(pricerdata->vertex_branching_conshdlr);
    auto vertex_branching_conss = SCIPconshdlrGetConss(pricerdata->vertex_branching_conshdlr);
    debug_assert(n_vertex_branching_conss == 0 || vertex_branching_conss);
    const auto n_length_branching_conss = SCIPconshdlrGetNConss(pricerdata->length_branching_conshdlr);
    auto length_branching_conss = SCIPconshdlrGetConss(pricerdata->length_branching_conshdlr);
    debug_assert(n_length_branching_conss == 0 || length_branching_conss);

    // Get the low-level solver.
    auto& astar = SCIPprobdataGetAStar(probdata);

    // Get data from the low-level solver.
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
    ] = astar.data();

    // Print used paths.
#ifdef PRINT_DEBUG
    print_used_paths(scip);
#endif

    // Find the makespan.
    Timepoint makespan = 0;
    for (auto var : vars)
    {
        // Get the path length.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto path_length = SCIPvardataGetPathLength(vardata);

        // Store the length of the longest path.
        if (path_length > makespan)
        {
            makespan = path_length;
        }
    }

    // Print dual values.
//#ifdef PRINT_DEBUG
//    print_agent_part_dual(scip, is_farkas);
//    print_vertex_conflicts_dual(scip, is_farkas);
//    print_edge_conflicts_dual(scip, is_farkas);
//    print_two_agent_robust_cuts_dual(scip, is_farkas);
//#ifdef USE_GOAL_CONFLICTS
//    print_goal_conflicts_dual(scip, is_farkas);
//#endif
//#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
//    print_rectangle_clique_conflicts_dual(scip, is_farkas);
//#endif
//#endif

    // Set up reservation table. Reserve vertices of paths with value 1.
#ifdef USE_RESERVATION_TABLE
    auto& restab = astar.reservation_table();
    restab.clear_reservations();
    for (auto var : vars)
    {
        debug_assert(var);
        const auto var_val = SCIPgetSolVal(scip, nullptr, var);
        if (var_val >= 0.5)
        {
            // Get the path.
            auto vardata = SCIPvarGetData(var);
            const auto path_length = SCIPvardataGetPathLength(vardata);
            const auto path = SCIPvardataGetPath(vardata);

            // Update reservation table.
            Location n;
            Timepoint t = 0;
            for (; t < path_length; ++t)
            {
                n = path[t].n;
                restab.reserve(LocationTimepoint{n, t});
            }
            for (; t < makespan; ++t)
            {
                restab.reserve(LocationTimepoint{n, t});
            }
        }
    }
#endif

    // Make edge penalties for all agents.
    EdgePenalties global_edge_penalties;

    // Input dual values for vertex conflicts.
    for (const auto& [nt, vertex_conflict] : vertex_conflicts_conss)
    {
        const auto& [row] = vertex_conflict;
        const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
        debug_assert(SCIPisFeasLE(scip, dual, 0.0));
        if (SCIPisFeasLT(scip, dual, 0.0))
        {
            // Add the dual variable value to the edges leading into the vertex.
            const auto t = nt.t - 1;
            {
                const auto n = map.get_south(nt.n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, t);
                penalties.north -= dual;
            }
            {
                const auto n = map.get_north(nt.n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, t);
                penalties.south -= dual;
            }
            {
                const auto n = map.get_west(nt.n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, t);
                penalties.east -= dual;
            }
            {
                const auto n = map.get_east(nt.n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, t);
                penalties.west -= dual;
            }
            {
                const auto n = map.get_wait(nt.n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, t);
                penalties.wait -= dual;
            }
        }
    }

    // Input dual values for edge conflicts.
    for (const auto& [et, edge_conflict] : edge_conflicts_conss)
    {
        const auto& [row, edges, t] = edge_conflict;
        const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
        debug_assert(SCIPisFeasLE(scip, dual, 0.0));
        if (SCIPisFeasLT(scip, dual, 0.0))
        {
            // Add the dual variable value to the edges.
            for (const auto e : edges)
            {
                auto& penalties = global_edge_penalties.get_edge_penalties(e.n, t);
                penalties.d[e.d] -= dual;
            }
        }
    }

    // Price each agent.
    Float min_reduced_cost = 0;
#ifdef PRINT_DEBUG
    Int nb_new_cols = 0;
#endif
    bool found = false;
    auto agent_priced = pricerdata->agent_priced;
    for (Int order_idx = 0;
         order_idx < N && (!found || order[order_idx].must_price) && !SCIPisStopped(scip);
         ++order_idx)
    {
        // Create output.
        Vector<Edge> path;
        Vector<LocationTimepoint> path_vertices;
        SCIP_Real path_cost;

        // Set up start and end points.
        const auto a = order[order_idx].a;
        start = agents[a].start;
        goal = agents[a].goal;

        // Input the agent partition dual.
        {
            // Get the constraint.
            auto cons = agent_part[a];
            debug_assert(cons);

            // Check that the constraint is not (locally) disabled/redundant.
            debug_assert(SCIPconsIsEnabled(cons));

            // Check that no variable is fixed to one.
            debug_assert(SCIPgetNFixedonesSetppc(scip, cons) == 0);

            // Store dual value.
            const auto dual = is_farkas ? SCIPgetDualfarkasSetppc(scip, cons) : SCIPgetDualsolSetppc(scip, cons);
            debug_assert(SCIPisGE(scip, dual, 0.0));
            cost_offset = -dual;
        }

        // Modify edge costs for two-agent robust cuts.
        edge_penalties = global_edge_penalties;
        finish_time_penalties.clear();
#ifdef USE_GOAL_CONFLICTS
        goal_penalties.clear();
#endif
        for (const auto& [row, ets_begin, ets_end] : agent_robust_cuts[a])
        {
            const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
            debug_assert(SCIPisFeasLE(scip, dual, 0.0));
            if (SCIPisFeasLT(scip, dual, 0.0))
            {
                for (auto it = ets_begin; it != ets_end; ++it)
                {
                    // Incur the penalty on the edge.
                    const auto n = it->n;
                    const auto d = it->d;
                    const auto t = it->t;
                    auto& penalties = edge_penalties.get_edge_penalties(n, t);
                    penalties.d[d] -= dual;

                    // If a wait edge in a two-agent robust cut corresponds to waiting at the goal, incur the
                    // penalty for staying at the goal because the low-level solver doesn't traverse this edge.
                    if (n == goal && d == Direction::WAIT)
                    {
                        const auto conflict_time = it->t;
                        finish_time_penalties.add(conflict_time, -dual);
                    }
                }
            }
        }

        // If the node of a vertex conflict is the goal, incur a penalty for waiting (indefinitely) at the goal
        // after the agent has completed its path.
        for (const auto& [t, row] : agent_goal_vertex_conflicts[a])
        {
            const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
            debug_assert(SCIPisFeasLE(scip, dual, 0.0));
            if (SCIPisFeasLT(scip, dual, 0.0))
            {
                // Add the penalty if the agent finishes before time t. The penalty at time t is accounted for in the
                // global edge penalties.
                finish_time_penalties.add(t - 1, -dual);
            }
        }

        // If the wait edge in a wait edge conflict is at the goal, incur a penalty for staying at the goal.
#ifdef USE_WAITEDGE_CONFLICTS
        for (const auto& [t, row] : agent_goal_edge_conflicts[a])
        {
            const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
            debug_assert(SCIPisFeasLE(scip, dual, 0.0));
            if (SCIPisFeasLT(scip, dual, 0.0))
            {
                // If an agent finishes at time t, it does not traverse the edge at time t to t+1. So the agent is 
                // penalised here.
                finish_time_penalties.add(t, -dual);
            }
        }
#endif

        // Add goal crossings or finish time penalties for goal conflicts. If agent a1 finishes at or before time t,
        // incur the penalty. If agent a2 crosses the goal of agent a1 at or after time t, incur the penalty.
#ifdef USE_GOAL_CONFLICTS
        for (const auto& [t, row] : goal_agent_goal_conflicts[a])
        {
            const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
            debug_assert(SCIPisFeasLE(scip, dual, 0.0));
            if (SCIPisFeasLT(scip, dual, 0.0))
            {
                finish_time_penalties.add(t, -dual);
            }
        }
        for (const auto& [nt, row] : crossing_agent_goal_conflicts[a])
        {
            const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
            debug_assert(SCIPisFeasLE(scip, dual, 0.0));
            if (SCIPisFeasLT(scip, dual, 0.0))
            {
                goal_penalties.add(nt, -dual);
            }
        }
#endif

        // Modify edge costs for path length nogoods. If agent a finishes at or before time t, incur the penalty.
#ifdef USE_PATH_LENGTH_NOGOODS
        for (const auto& [row, latest_finish_times] : path_length_nogoods)
            for (const auto& [nogood_a, t] : latest_finish_times)
                if (a == nogood_a)
                {
                    const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
                    debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                    if (SCIPisFeasLT(scip, dual, 0.0))
                    {
                        finish_time_penalties.add(t, -dual);
                    }
                }
#endif

        // Modify edge costs for vertex branching decisions.
        waypoints.clear();
        for (Int c = 0; c < n_vertex_branching_conss; ++c)
        {
            // Get the constraint.
            auto cons = vertex_branching_conss[c];
            debug_assert(cons);

            // Ignore constraints that are not active since these are not on the current
            // active path of the search tree.
            if (!SCIPconsIsActive(cons))
                continue;

            // Enforce the decision.
            const auto branch_a = SCIPgetVertexBranchingRobot(cons);
            const auto dir = SCIPgetVertexBranchingDirection(cons);
            const auto nt = SCIPgetVertexBranchingLocationTimepoint(cons);
            if ((a == branch_a && dir == VertexBranchDirection::Forbid) ||
                (a != branch_a && dir == VertexBranchDirection::Use))
            {
                // Don't use the vertex.
                const auto prev_time = nt.t - 1;
                {
                    const auto n = map.get_south(nt.n);
                    auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                    penalties.north = std::numeric_limits<Cost>::infinity();
                }
                {
                    const auto n = map.get_north(nt.n);
                    auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                    penalties.south = std::numeric_limits<Cost>::infinity();
                }
                {
                    const auto n = map.get_west(nt.n);
                    auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                    penalties.east = std::numeric_limits<Cost>::infinity();
                }
                {
                    const auto n = map.get_east(nt.n);
                    auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                    penalties.west = std::numeric_limits<Cost>::infinity();
                }
                {
                    const auto n = map.get_wait(nt.n);
                    auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                    penalties.wait = std::numeric_limits<Cost>::infinity();
                }
            }
            else if (a == branch_a)
            {
                debug_assert(dir == VertexBranchDirection::Use);

                // Store a waypoint to enforce use of the vertex.
                waypoints.push_back(nt);
            }
        }

        // Sort waypoints by time.
        std::sort(waypoints.begin(), waypoints.end(), [](const auto& a, const auto& b)
        {
            return a.t < b.t;
        });
#ifdef DEBUG
        for (size_t idx = 1; idx < waypoints.size(); ++idx)
        {
            debug_assert(waypoints[idx - 1].t < waypoints[idx].t);
        }
#endif

        // Modify edge costs for length branching decisions.
        debug_assert(astar.max_path_length() >= 1);
        earliest_goal_time = 0;
        latest_goal_time = astar.max_path_length() - 1;
        latest_visit_time = map.latest_visit_time();
        for (Int c = 0; c < n_length_branching_conss; ++c)
        {
            // Get the constraint.
            auto cons = length_branching_conss[c];
            debug_assert(cons);

            // Ignore constraints that are not active since these are not on the current active path of the search tree.
            if (!SCIPconsIsActive(cons))
                continue;

            // Enforce the decision if the same agent. Disable crossing if different agent.
            const auto branch_a = SCIPgetLengthBranchingRobot(cons);
            const auto dir = SCIPgetLengthBranchingDirection(cons);
            const auto nt = SCIPgetLengthBranchingLocationTimepoint(cons);
            if (a == branch_a)
            {
                if (dir == LengthBranchDirection::LEq)
                {
                    latest_goal_time = std::min(latest_goal_time, nt.t);
                }
                else
                {
                    earliest_goal_time = std::max(earliest_goal_time, nt.t);
                }
            }
            else if (dir == LengthBranchDirection::LEq)
            {
                // Block the vertex at time t and onwards.
                latest_visit_time[nt.n] = nt.t - 1;
            }
        }
        debug_assert(waypoints.empty() || latest_goal_time >= waypoints.back().t);

        // Preprocess input data.
        astar.preprocess_input();

        // Start timer.
#ifdef PRINT_DEBUG
        const auto start_time = std::chrono::high_resolution_clock::now();
#endif

        // Skip running A* if the penalties in the last iteration of this agent have stayed the same or worsened.
#ifdef USE_ASTAR_SOLUTION_CACHING
        if (!astar.data().can_be_better(pricerdata->previous_data[a]))
        {
            goto FINISHED_PRICING_AGENT;
        }
#endif

        // Solve.
        std::tie(path_vertices, path_cost) = astar.solve<is_farkas>();
        if (!path_vertices.empty())
        {
            // Get the solution.
            for (auto it = path_vertices.begin(); it != path_vertices.end(); ++it)
            {
                const auto d = it != path_vertices.end() - 1 ?
                               map.get_direction(it->n, (it + 1)->n) :
                               Direction::INVALID;
                path.push_back(Edge{it->n, d});
            }

            // Add a column only if the path has negative reduced cost.
            min_reduced_cost = std::min(min_reduced_cost, path_cost);
            if (SCIPisSumLT(scip, path_cost, 0.0))
            {
                // Print.
                debugln("    Found path with length {}, reduced cost {:.6f} ({})",
                        path.size(),
                        path_cost,
                        format_path(probdata, path.size(), path.data()));

                // Add column.
                SCIP_VAR* var = nullptr;
                SCIP_CALL(SCIPprobdataAddPricedVar(scip, probdata, a, path.size(), path.data(), &var));
                debug_assert(var);
                found = true;
                order[order_idx].new_var = var;
                pricerdata->price_priority[a]++;
#ifdef PRINT_DEBUG
                nb_new_cols++;
#endif

#ifdef USE_RESERVATION_TABLE
                // Update reservation table.
                {
                    Location n;
                    Timepoint t = 0;
                    for (; t < static_cast<Timepoint>(path.size()); ++t)
                    {
                        n = path[t].n;
                        restab.reserve(LocationTimepoint{n, t});
                    }
                    for (; t < makespan; ++t)
                    {
                        restab.reserve(LocationTimepoint{n, t});
                    }
                }
#endif

                // Advance to the next agent.
                goto FINISHED_PRICING_AGENT;
            }
        }

        // Store the penalties of the run.
#ifdef USE_ASTAR_SOLUTION_CACHING
        pricerdata->previous_data[a] = astar.data();
#endif

        // End of this agent.
        FINISHED_PRICING_AGENT:
        agent_priced[a] = true;

        // End timer.
#ifdef PRINT_DEBUG
        const auto end_time = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration<double>(end_time - start_time).count();
        debugln("    Done in {:.4f} seconds", duration);
#endif
    }

    // Print.
    debugln("Added {} new columns", nb_new_cols);

    // Finish.
    if (!SCIPisStopped(scip))
    {
        // Compute lower bound.
        if constexpr (!is_farkas)
        {
            bool all_agents_priced = true;
            for (Robot a = 0; a < N; ++a)
                if (!agent_priced[a])
                {
                    all_agents_priced = false;
                    break;
                }
            if (all_agents_priced)
            {
                *lower_bound = SCIPgetLPObjval(scip) + N * min_reduced_cost;
                debugln("   Computed lower bound {}", *lower_bound);
            }
        }

        // Mark as completed.
        *result = SCIP_SUCCESS;
    }
    return SCIP_OKAY;
}

// Reduced cost pricing for feasible master problem
static
SCIP_DECL_PRICERREDCOST(pricerTruffleHogRedCost)
{
    if ( SCIPnodeGetNumber(SCIPgetCurrentNode(scip)) == 1)
        return run_full_pricer(scip, pricer, result, stopearly, lowerbound); 
    else 
        return run_trufflehog_pricer(scip, pricer, result, stopearly, lowerbound);
}

// Farkas pricing for infeasible master problem
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_PRICERFARKAS(pricerTruffleHogFarkas)
{
    unreachable();
}
#pragma GCC diagnostic pop

// Create pricer and include it in SCIP
SCIP_RETCODE SCIPincludePricerTruffleHog(
    SCIP* scip    // SCIP
)
{
    // Include pricer.
    SCIP_PRICER* pricer;
    SCIP_CALL(SCIPincludePricerBasic(scip,
                                     &pricer,
                                     PRICER_NAME,
                                     PRICER_DESC,
                                     PRICER_PRIORITY,
                                     PRICER_DELAY,
                                     pricerTruffleHogRedCost,
                                     pricerTruffleHogFarkas,
                                     nullptr));

    // Set callbacks.
    SCIP_CALL(SCIPsetPricerInit(scip, pricer, pricerTruffleHogInit));
    SCIP_CALL(SCIPsetPricerFree(scip, pricer, pricerTruffleHogFree));

    // Done.
    return SCIP_OKAY;
}

// Add problem specific data to the pricer and activate
SCIP_RETCODE SCIPpricerTruffleHogActivate(
    SCIP* scip    // SCIP
)
{
    // Check.
    debug_assert(scip);

    // Get pricer.
    auto pricer = SCIPfindPricer(scip, PRICER_NAME);
    debug_assert(pricer);

    // Activate pricer.
    SCIP_CALL(SCIPactivatePricer(scip, pricer));

    // Done.
    return SCIP_OKAY;
}

// Inject a warm-start solution
SCIP_RETCODE add_initial_solution(
    SCIP* scip    // SCIP
)
{
    return SCIP_OKAY;
}
