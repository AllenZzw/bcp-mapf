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

#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS

#ifndef MAPF_SEPARATOR_RECTANGLECLIQUECONFLICTS_H
#define MAPF_SEPARATOR_RECTANGLECLIQUECONFLICTS_H

#include "Includes.h"
#include "Coordinates.h"
#include "ProblemData.h"
#include "Separator_RectangleConflicts.h"

// Create separator for rectangle clique conflicts and include it
SCIP_RETCODE SCIPincludeSepaRectangleCliqueConflicts(
    SCIP* scip,         // SCIP
    SCIP_SEPA** sepa    // Output pointer to separator
);

SCIP_RETCODE rectangle_clique_conflicts_add_var(
    SCIP* scip,                // SCIP
    SCIP_SEPA* sepa,           // Separator for rectangle clique conflicts
    SCIP_VAR* var,             // Variable
    const Agent a,             // Agent
    const Time path_length,    // Path length
    const Edge* const path     // Path
);

const Vector<RectangleConflict>& rectangle_clique_conflicts_get_constraints(
    SCIP_ProbData* probdata    // Problem data
);

#endif

#endif
