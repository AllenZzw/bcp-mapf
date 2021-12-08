BCP-LNS
===

Adding large neighbourhood search (LNS) for a branch-and-cut-and-price (BCP) model of the multi-agent path finding problem. The BCP solver is described in the following papers: 
- Branch-and-Cut-and-Price for Multi-Agent Pathfinding. E. Lam, P. Le Bodic, D. Harabor, P. J. Stuckey. IJCAI 2019.
- New Valid Inequalities in Branch-and-Cut-and-Price for Multi-Agent Path Finding. E. Lam, P. Le Bodic. ICAPS 2020.
The LNS solver is described in the following paper: 
- Jiaoyang Li, Zhe Chen, Daniel Harabor, Peter J. Stuckey, Sven Koenig. Anytime Multi-Agent Path Finding via Large Neighborhood Search. In Proceedings of the International Joint Conference on Artificial Intelligence (IJCAI), (in print), 2021.
- Jiaoyang Li, Zhe Chen, Daniel Harabor, Peter J. Stuckey, Sven Koenig. Anytime Multi-Agent Path Finding via Large Neighborhood Search: Extended Abstract. In Proceedings of the International Conference on Autonomous Agents and Multiagent Systems (AAMAS), pages 1581-1583, 2021.

Please go to the following respository for the original source code: 
- BCP: https://github.com/ed-lam/bcp-mapf/
- LNS: https://github.com/Jiaoyang-Li/MAPF-LNS

License
-------

BCP is released under the GPL version 3. See LICENSE.txt for further details. 

Dependencies
------------

BCP is implemented in C++17 and is compiled using CMake, so you will need a recent compiler and a recent version of CMake. It is tested with Clang 11 on Mac and GCC 10 on Linux. It has not been tested on Windows.

BCP calls SCIP for branch-and-bound and calls CPLEX for solving the linear relaxation.

Source code to SCIP is available free (as in beer) strictly for academic use. BCP is tested with SCIP 7.0.3. Download the [SCIP Optimization Suite 7.0.3](https://scip.zib.de) and extract it into the root of this repository. You should find the subdirectory `scipoptsuite-7.0.3/scip/src`.

CPLEX is commercial software but has binaries available free under an [academic license](https://community.ibm.com/community/user/datascience/blogs/xavier-nodet1/2020/07/09/cplex-free-for-students). BCP is tested with CPLEX 12.10. You should find the subdirectory `cplex`.

If CPLEX is not available, SoPlex from the SCIP Optimization Suite can be used instead but this option is not supported.

The LNS solver will additionally requires the external libraries BOOST (https://www.boost.org/) and Eigen (https://eigen.tuxfamily.org/). 

Compiling
---------

Download the source code by cloning this Git repository and all its submodules:
```
git clone --recurse-submodules https://github.com/ed-lam/bcp-mapf.git
```

Locate the `cplex` subdirectory inside wherever you downloaded the CPLEX binaries. Compile BCP using CMake:
```
cd bcp-mapf
mkdir build
cd build
cmake -DCPLEX_DIR={PATH TO cplex SUBDIRECTORY} ..
cmake --build .
```

If you use a custom compiler, you will need to tell CMake where the compiler is:
```
cmake -DCPLEX_DIR={PATH TO cplex SUBDIRECTORY} -DCMAKE_C_COMPILER={PATH TO C COMPILER} -DCMAKE_CXX_COMPILER={PATH TO C++ COMPILER} ..
```

If you have a multi-core CPU with N cores, you can perform a parallel compile by running the following command instead, replacing N with the number of cores:
```
cmake --build . -j N
```

Usage
-----

After compiling, run BCP with:
```
./bcp-mapf -f {PATH TO INSTANCE} -m {PATH TO MAP}
```

You can also set a time limit in seconds:
```
./bcp-mapf —-time-limit={TIME LIMIT} -f {PATH TO INSTANCE} -m {PATH TO MAP}
```

BCP can be run as a bounded suboptimal algorithm by setting an optimality gap, calculated as (upper bound - lower bound) / lower bound. For example, enter `0.1` for a 10% optimality gap.
```
./bcp-mapf —g {OPTIMALITY GAP} -f {PATH TO INSTANCE} -m {PATH TO MAP}
```

Benchmark instances can be found in the `movingai_2018` and `movingai_2019` directories. Example:
```
./bcp-mapf --time-limit=30 --agents-limit=50 -f ../instances/movingai_2019/den520d-random-1.scen -m ../instances/movingai_2019/den520d.map
```

The optimal solution (or feasible solution if a time limit or gap limit is reached) will be saved into the `outputs` directory.


Known Issues
-----
- Currently, the solutions from the LNS solver and the BCP solver are not synchronized. The primal solution of BCP is updated mainly by the simple rounding primal heuristic. 
