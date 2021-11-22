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

#ifndef TRUFFLEHOG_PROBLEMINSTANCE_H
#define TRUFFLEHOG_PROBLEMINSTANCE_H

#include "AgentsData.h"
#include "Map.h"

namespace TruffleHog
{

struct ProblemInstance
{
    Map map;
    AgentsData agents;

  public:
    // Constructors
    ProblemInstance() = default;
    ProblemInstance(const char* scenario_path, const Agent nb_agents = std::numeric_limits<Agent>::max());
    ProblemInstance(const ProblemInstance&) = default;
    ProblemInstance(ProblemInstance&&) = default;
    ProblemInstance& operator=(const ProblemInstance&) = default;
    ProblemInstance& operator=(ProblemInstance&&) = default;
    ~ProblemInstance() = default;
};

}

#endif
