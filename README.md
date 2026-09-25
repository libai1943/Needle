# Flexible Needle Trajectory Planning

MATLAB implementation of the offline trajectory planning module developed for:

> **Computational Optimal Control for Flexible Needle Trajectory Planning and Online Replanning**  
> Bai Li, Yakun Ouyang, Zhuyan Yin, et al.

This repository currently provides the **offline trajectory planning** part of the framework, including trajectory generation, nonlinear refinement, trajectory validation, visualization, animation, and inspection of optimized state and control variables.

The code is intended primarily for research and reproducibility.

---

## Overview

The framework considers three-dimensional trajectory planning for a flexible bevel-tip needle moving through an obstacle-filled workspace.

The overall study contains two connected components:

- **Offline trajectory planning:** explore multiple geometrically different insertion routes and refine them into dynamically feasible needle trajectories.
- **Online trajectory replanning:** update the remaining trajectory when the measured needle state deviates from the nominal trajectory or when the obstacle configuration changes.

The code released in this repository focuses on the **offline component**.

---

## Offline Planning

A direct nonlinear trajectory optimization can be sensitive to its initial guess, especially in cluttered environments. The offline planner therefore does not rely on a single geometric initialization.

The main procedure is:

1. **Generate alternative geometric routes**

   Repeated graph searches are performed in the three-dimensional workspace. Previously explored regions are progressively penalized so that subsequent searches tend to discover different obstacle-avoidance routes.

2. **Select representative routes**

   The generated routes are resampled according to normalized arc length and compared geometrically. A limited number of mutually different routes are selected as representatives, avoiding the need to optimize every graph-search result.

3. **Fit the needle model**

   Each representative geometric route is converted into an initialization compatible with the flexible-needle motion model.

4. **Perform nonlinear trajectory optimization**

   The needle trajectory is refined under the motion model, boundary conditions, actuation limits, workspace constraints, and obstacle-avoidance constraints.

5. **Validate the optimized trajectory**

   Candidate trajectories are independently checked after optimization, including geometric obstacle clearance, numerical trajectory replay, terminal accuracy, and nonlocal needle-body collision checks.

6. **Select the final trajectory**

   Multiple accepted local solutions may remain. In addition to the optimization objective, an external obstacle-proximity field evaluates how strongly the complete trajectory is influenced by nearby obstacles. The final trajectory is selected from the accepted candidate set.

In short, the offline planner follows

```text
Multiple geometric routes
        ↓
Representative selection
        ↓
Needle-model fitting
        ↓
Nonlinear trajectory refinement
        ↓
Independent trajectory validation
        ↓
Final trajectory selection
