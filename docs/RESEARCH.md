# Research review and algorithm selection

Dates are business dates in UTC+8. This record explains why the Phase 1
candidates use the methods they use. Bibliographic details were checked
against Crossref and arXiv on 2026-10-13; the references section notes every
correction to the citations as first supplied.

## 1. What the simulator demands

The choice of methods follows from measured properties of the supplied
simulator (details in [AGENT_DESIGN.md](AGENT_DESIGN.md), section 1):

* **Deterministic, linear dynamics.** Thrust and rotation responses fit
  exactly, so a model-based controller and dead reckoning are reliable as long
  as nothing is in contact. That favours classical estimation and control over
  learned dynamics.
* **Slowly varying absolute errors.** GPS and compass noise are AR(1) with
  phi 0.98: averaging over a few steps does not help, and pose accuracy is
  about 5-7 px unless the map itself is used for localization.
* **Short communication range (250 px), one-step latency, no global state.**
  Coordination has to tolerate partitions and stale information.
* **A tight compute budget.** Wall-clock limits of 90-240 s for 2000-7200
  steps and 10 drones leave a few milliseconds per drone and step in Python;
  the evaluator's hardware is unknown.
* **Unknown official maps (D4).** Nothing may depend on a particular map.
* **Scoring.** Blue: disposal 60 %, health 20 %, time 20 % (full time credit
  only when every bomb is disposed by half the step limit). Red: map accuracy
  50 % (exact overlap with the wall surface, penalty beyond 10 px), bomb
  difficulty 20 % (computed against organizer reference agents), wall
  coverage 20 %, time 10 %.

## 2. Method families assessed

| Family | Relevance here | Cost and risk | Decision |
|---|---|---|---|
| Occupancy grids with log-odds updates (Elfes 1989) | Direct fit for lidar mapping and for planning; robust to noisy returns. | Cheap at coarse resolution; fine resolution needed only for the red map. | **Adopted**: 8 px log-odds grid for navigation, 1 px evidence counts for the red map. |
| Frontier-based exploration (Yamauchi 1997, 1998) | Systematic coverage of unknown maps; the multi-robot form shares sensor data but decides independently, which matches the radio model. | Simple; redundancy between robots unless utilities are coordinated. | **Adopted**, with search-range targets. |
| Coordinated utility-cost target assignment (Burgard et al. 2005) | Reduces overlap by discounting targets near teammates' targets; has a limited-range variant. | Cheap; needs shared target information. | **Partly adopted**: teammates' targets and positions penalize candidates. Sharing of covered areas is the next step (defect 2). |
| Market-based and consensus allocation (Dias et al. 2006; Choi, Brunet, How 2009, CBBA; taxonomy: Gerkey, Mataric 2004) | Bomb retrieval is single-task robots / single-robot tasks with instantaneous assignment; auctions give conflict-free assignments under inconsistent knowledge. | Full CBBA adds message rounds and bookkeeping. | **Simplified form adopted**: greedy claims with cost-based precedence and relayed claims. CBBA is the upgrade path if claim conflicts appear in real runs. |
| Geodesic planning: A* (Hart, Nilsson, Raphael 1968) and Dijkstra fields | Collision-aware routes on the grid; one field serves many queries. | Pure-Python search is too slow; compiled Dijkstra (`skimage.graph.MCP_Geometric`) computes a full field in about 3 ms. | **Adopted**: compiled fields with an OpenCV wavefront fallback. |
| Velocity-space reactive avoidance: dynamic window (Fox, Burgard, Thrun 1997), reciprocal avoidance ORCA (van den Berg et al. 2011) | Fast local avoidance using the robot's dynamics. | DWA sampling and ORCA linear programs cost more than a deadbeat inversion of an exactly known model. | **Simplified form adopted**: deadbeat velocity control, braking-distance speed limits, repulsion. ORCA is a candidate if drone-drone contacts become a measurable health cost. |
| Multi-agent path finding, CBS (Sharon et al. 2015) | Optimal joint paths for many agents on graphs. | Exponential in conflicts, assumes synchronized discrete moves and full knowledge; neither holds here. | **Rejected** for this simulator. |
| Joint pose and map optimization, e.g. Occupancy-SLAM (Wang, Zhao, Huang 2025) and scan matching | Map-based localization is the only route to pose errors well below the GPS level, which matters for the red map score. | Optimization is too heavy per step; correlative scan-to-map matching every few steps is affordable. | **Deferred** to R1; the estimator design leaves room for it. |
| Rendezvous-based map merging under limited communication (Meeting-Merging-Mission, Gao et al., arXiv 2109.07764) | Matches the 250 px radio: robots meet to merge maps and redistribute goals. | Meetings cost exploration time. | **Simplified form adopted**: a final rally before submission; scheduled meetings are a candidate if snapshots arrive late. |
| Deep RL, multi-agent RL, graph networks, transformers, LLM inference (survey: Al Mahmud et al. 2025) | Could learn allocation or exploration policies. | Needs a fast simulator, much training, and must beat a working baseline; adds runtime and dependency risk (no PyTorch in the evaluator image). | **Rejected for now**: no evidence it would beat the classical baseline within the compute budget. |

## 3. Selected architecture

A modular classical pipeline per drone (estimation, mapping, planning,
reactive control) with decentralized coordination over gossiped messages,
and role-specific policies on top: claim, fetch and deliver for blue;
explore, place by greedy coverage and submit for red. The survey by Wang et
al. (arXiv 2503.07278) organizes cooperative exploration systems into the
same building blocks (localization and mapping, map merging, cooperative
planning, communication), which supports this decomposition. Every component
has a fallback when its input is missing (dead reckoning without GPS or
compass, local decisions without messages, direct steering without a route).

Rejected for this cycle: several competing architectures in parallel (no
measurements yet to choose between them), learning-based policies, and
optimal multi-agent path planning.

## 4. References

Status after verification on 2026-10-13. Contribution notes are limited to
what the abstracts or records state.

1. C. Wang, C. Yu, X. Xu, Y. Gao, X. Yang, W. Tang, S. Yu, Y. Chen, F. Gao,
   Z. Jian, X. Chen, F. Gao, B. Zhou, Y. Wang. *Multi-Robot System for
   Cooperative Exploration in Unknown Environments: A Survey.*
   arXiv:2503.07278, 2025. Verified.
2. B. Yamauchi. *Decentralized coordination for multirobot exploration.*
   Robotics and Autonomous Systems 29(2-3):111-118, 1999.
   doi:10.1016/S0921-8890(99)00046-9. Verified (no abstract available from
   the indexes; cited for its topic only).
3. G. Sharon, R. Stern, A. Felner, N. R. Sturtevant. *Conflict-based search
   for optimal multi-agent pathfinding.* Artificial Intelligence 219:40-66,
   **2015**. doi:10.1016/j.artint.2014.11.006. The DOI contains "2014"; the
   issue is dated February 2015.
4. S. Al Mahmud, A. M. Ibrahim, M. R. Khan. *A comprehensive review on multi
   robot task assignment and path planning: Operational constraints,
   challenges, and potentials of deep reinforcement learning.* Applied Soft
   Computing 188:114498 (issued 2025-12-23, print issue February 2026).
   doi:10.1016/j.asoc.2025.114498. The title as first supplied to this
   project was a shortened paraphrase; the indexed title is given here.
5. B. Yamauchi. *A frontier-based approach for autonomous exploration.* Proc.
   IEEE CIRA 1997, pp. 146-151. doi:10.1109/CIRA.1997.613851.
6. B. Yamauchi. *Frontier-based exploration using multiple robots.* Proc.
   AGENTS 1998, pp. 47-53. doi:10.1145/280765.280773.
7. W. Burgard, M. Moors, C. Stachniss, F. E. Schneider. *Coordinated
   multi-robot exploration.* IEEE Transactions on Robotics 21(3):376-386,
   2005. doi:10.1109/TRO.2004.839232.
8. A. Elfes. *Using occupancy grids for mobile robot perception and
   navigation.* Computer 22(6):46-57, 1989. doi:10.1109/2.30720.
9. P. E. Hart, N. J. Nilsson, B. Raphael. *A formal basis for the heuristic
   determination of minimum cost paths.* IEEE Transactions on Systems Science
   and Cybernetics 4(2):100-107, 1968. doi:10.1109/TSSC.1968.300136.
10. D. Fox, W. Burgard, S. Thrun. *The dynamic window approach to collision
    avoidance.* IEEE Robotics & Automation Magazine 4(1):23-33, 1997.
    doi:10.1109/100.580977.
11. B. P. Gerkey, M. J. Mataric. *A formal analysis and taxonomy of task
    allocation in multi-robot systems.* International Journal of Robotics
    Research 23(9):939-954, 2004. doi:10.1177/0278364904045564.
12. M. B. Dias, R. Zlot, N. Kalra, A. Stentz. *Market-based multirobot
    coordination: a survey and analysis.* Proceedings of the IEEE
    94(7):1257-1270, 2006. doi:10.1109/JPROC.2006.876939.
13. J. van den Berg, S. J. Guy, M. Lin, D. Manocha. *Reciprocal n-body
    collision avoidance.* In Robotics Research (ISRR 2009), Springer Tracts in
    Advanced Robotics 70, pp. 3-19, 2011. doi:10.1007/978-3-642-19457-3_1.
14. H.-L. Choi, L. Brunet, J. P. How. *Consensus-based decentralized auctions
    for robust task allocation.* IEEE Transactions on Robotics 25(4):912-926,
    2009. doi:10.1109/TRO.2009.2022423.
15. Y. Gao, Y. Wang, X. Zhong, T. Yang, M. Wang, Z. Xu, Y. Wang, C. Xu,
    F. Gao. *Meeting-Merging-Mission: A Multi-robot Coordinate Framework for
    Large-Scale Communication-Limited Exploration.* arXiv:2109.07764, 2021
    (a conference venue was reported by a search engine but not confirmed).
16. Y. Wang, L. Zhao, S. Huang. *Occupancy-SLAM: An Efficient and Robust
    Algorithm for Simultaneously Optimizing Robot Poses and Occupancy Map.*
    arXiv:2502.06292, 2025 (accepted for IEEE Transactions on Robotics
    according to the arXiv record).
