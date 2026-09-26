# Flexible Needle Trajectory Planning and Online Replanning

MATLAB code accompanying the manuscript:

> **Computational Optimal Control for Flexible Needle Trajectory Planning and Online Replanning**  
> Bai Li, Yakun Ouyang, Zhuyan Yin, et al.

This repository is provided as supplementary material for peer review.

## Overview

The examples address three-dimensional trajectory planning for a flexible bevel-tip needle in environments containing obstacles. The repository includes offline planning, trajectory validation, visualization of states and controls, and optional animation.

The current version also includes **online replanning demonstrations** that update the remaining trajectory in response to simulated needle-state deviations or changes in obstacle configuration.

Please refer to the accompanying manuscript for the mathematical formulation, algorithmic details, and experimental analysis.

## Requirements

The supplied configuration uses:

- MATLAB.
- CasADi 3.6.1 with its MATLAB interface.
- IPOPT with HSL MA97 support.

MA97 must be available in the solver installation for the supplied configuration to run unchanged.

Set `CASADI_ROOT` to the directory containing the `+casadi` folder. For example:

```matlab
setenv('CASADI_ROOT', 'C:/casadi');  % Adjust to your local installation.
```

Open MATLAB in the repository root before running the examples below.

## Quick Start

### Offline planning

```matlab
[result, figures] = RunMe(1, false);
```

The first argument selects one of the 20 included cases. Set the second argument to `true` to generate a video.

### Online replanning

```matlab
[output, figures] = RunOnlineReplanner(1);
```

This runs a simulation example using random seed `1`, including offline preparation and an online replanning event. Use an explicit seed when comparing repeated runs.

## Results

Outputs are saved under `results/`. Offline runs save the planning results and, for accepted trajectories, trajectory plots and state/control histories. Online runs create a timestamped subfolder containing saved results and replanning visualizations.

Both entry points return result structures and figure handles for further inspection.

## Scope

This repository contains research software for simulation and peer-review evaluation. It is not intended for clinical use.
