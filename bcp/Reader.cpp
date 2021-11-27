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

#include "Reader.h"
#include "Includes.h"
#include "ProblemData.h"
#include <regex>
#include <filesystem>

#include "trufflehog/ProblemInstance.h"
#include "trufflehog/AStar.h"
#include "lns/inc/LNS.h"

// Read instance from file
SCIP_RETCODE read_instance(
    SCIP* scip,                   // SCIP
    const char* map_path,         // File path to scenario
    const char* scenario_path,    // File path to scenario
    const Robot nb_agents         // Number of agents to read
)
{
    // Get instance name.
    std::filesystem::path path(scenario_path);
    auto instance_name = path.filename().stem().string();
    if (nb_agents < std::numeric_limits<Robot>::max())
    {
        instance_name += fmt::format("-{}agents", nb_agents);
    }

    // Load instance.
    auto instance = std::make_shared<ProblemInstance>(map_path, scenario_path, nb_agents);

    // Load LNS instance 
    auto lns_instance = std::make_shared<Instance>(instance->map_path_str, instance->scenario_path_str, nb_agents);

    // Create pricing solver.
    auto astar = std::make_shared<AStar>(instance->map);

    // Create lns solver 
    PIBTPPS_option pipp_option;
    pipp_option.windowSize = 5;
    pipp_option.winPIBTSoft = true;
    SCIP_Real time_limit = 0;
    SCIPgetRealParam(scip, "limits/time", &time_limit);
    auto lns = std::make_shared<LNS>(*lns_instance, time_limit, "PP", "PP", "Adaptive", 5, 50000, 0, pipp_option);

    // Create the problem.
    SCIP_CALL(SCIPprobdataCreate(scip, instance_name.c_str(), instance, lns_instance, astar, lns));

    // Done.
    return SCIP_OKAY;
}
