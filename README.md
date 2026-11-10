# Flexible needle trajectory planning and online replanning

This repository accompanies the following research article. **If you use the planner, the benchmark cases, or the replanning implementation in your research, please cite the paper.**

> Bai Li, Yakun Ouyang, Zhuyan Yin, Xinwei Wang, and Yongxiao Tian. **Computational optimal control for flexible needle trajectory planning and online replanning.** *Journal of Computational Design and Engineering*.

[Download BibTeX](CITATION.bib) · [Citation metadata](CITATION.cff) · [Paper code release](https://github.com/libai1943/Needle/releases) · [Annotated animations](https://www.bilibili.com/video/BV1sfxzziEbz/)

[Download ZIP](https://github.com/libai1943/Needle/archive/refs/heads/main.zip) · [Run a benchmark](#plan-an-insertion) · [Replan after an event](#replan-after-one-event) · [Read the implementation](#how-the-matlab-functions-fit-together)

A bevel-tip needle steers by advancing and rotating about its shaft. A short spatial path around an obstacle is useful only if the needle can follow it and reach the target with admissible controls. This MATLAB implementation uses the time available before insertion to explore several routes and fit a needle motion to each. During insertion, it reuses the selected trajectory to correct a measured tip deviation or to plan a new remaining path after an obstacle moves.

![Graphical abstract showing alternative routes around obstacles, the bevel-tip needle, and replanning after a disturbance](media/graphical-abstract.jpg)

## Plan an insertion

Download and extract the repository, then open its root directory in MATLAB. The examples were tested with **MATLAB R2021b** and the **CasADi 3.6.1 MATLAB interface**, using IPOPT with the **HSL MA97** linear solver. If CasADi is already on your MATLAB path, start at the second line. Otherwise, set `CASADI_ROOT` to the directory containing `+casadi`.

```matlab
setenv('CASADI_ROOT', 'C:/casadi')
[result, figures] = RunMe(1, false);
```

Case 1 has four spherical obstacles. The first figure shows the insertion entry, target, and selected centerline. The second traces the three position components, azimuth, elevation, axial roll, insertion speed, acceleration, and axial rotation rate. The bottom-right panel shows the rotation command, which can be compared directly with the axial-roll trace above it.

<p align="center">
  <a href="results/1_trajectory.png"><img src="results/1_trajectory.png" width="45%" alt="Case 1. The pink centerline passes between the entry and target around four dark blue spheres."></a>
  <a href="results/1_variables.png"><img src="results/1_variables.png" width="51%" alt="Case 1. Nine state and control histories plotted against insertion time."></a>
</p>

In a run of this command, insertion took **19.438 s** along the planned motion. The motion cost and selection score were both **19.878 s**, and the lower clearance estimate from kinematic replay was **5.914 mm**. MATLAB reports the wall time spent finding the plan separately. The numerical result is saved as `results/1.mat`; each figure is saved as both a PNG and an editable MATLAB FIG.

To see how the needle turns, run `MakeVideo(1)` after solving case 1. The left panel follows insertion as the camera rotates. The right panel isolates the needle-tip coordinate frame, so axial roll remains visible even when viewed along the path. Set the second argument of `RunMe` to `true` to solve and render in one call.

[![Animated preview of case 1 showing the advancing centerline and rotating local tip frame](media/insertion-preview.webp)](videos/1.mp4)

[Full-resolution case 1 video (1080p MP4)](videos/1.mp4)

## Explore the twenty benchmarks

[Cases.mat](Cases.mat) contains the entry, target, obstacle geometry, and model parameters for each fixed case. Pass the case number directly to [RunMe.m](RunMe.m).

| Case numbers | 1–4 | 5–8 | 9–12 | 13–16 | 17–20 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Spheres per case | 4 | 8 | 12 | 18 | 24 |

| Planning step | What happens in the code |
| --- | --- |
| Generate routes | [FindRoutes.m](FindRoutes.m) searches a 5 mm grid up to 48 times, increasing the cost of revisited cells. It samples each route at 40 arc-length locations and selects up to six geometrically distinct representatives for refinement. |
| Fit and refine | [OfflinePlan.m](OfflinePlan.m) fits the routes to the needle model and includes direct, single-route, and LIOM initializations. [BuildNLP.m](BuildNLP.m) forms a 400-node nonlinear program. Its motion cost combines insertion time, control effort, control variation, and an elevation-margin term. |
| Check a candidate | [CheckTrajectory.m](CheckTrajectory.m) checks the discrete constraints, segment-to-sphere clearance, fourth-order Runge–Kutta replay, and separation of nonlocal body segments. A clearance failure can trigger another solve with enlarged planning radii on the same mesh. |
| Select the trajectory | [RiskField.m](RiskField.m) integrates proximity to the physical obstacles along each accepted path. The planner adds the weighted proximity penalty to the motion cost and returns the candidate with the lowest combined score. |

**Case 20 shows why the two selection rules matter.** In the paper's comparison, selecting by the combined score reduced integrated obstacle proximity from 36.296 to 15.423 mm while increasing motion cost from 19.820 to 20.143 s. To inspect the two trajectories from your own run, compare the two rows below. The columns give motion cost (s), integrated proximity (mm), and selection score (s), respectively.

```matlab
r = RunMe(20, false);
byCost = r.min_cost;
byScore = r.best;
[byCost.cost,  byCost.risk.integral_mm,  byCost.selection_score
 byScore.cost, byScore.risk.integral_mm, byScore.selection_score]
```

## Replan after one event

[RunOnlineReplanner.m](RunOnlineReplanner.m) generates a new scene, solves its offline plan, and prepares a sensitivity library before applying **one** sudden event. The entry point samples the geometry, event time, and disturbance from a seed. The target remains fixed and the executed portion of the insertion is retained.

```matlab
[out, figures] = RunOnlineReplanner(260928);
out.event.kind
out.reply.action
out.folder
```

This seed produced an eight-sphere scene. At **6.633 s** a sphere moved into the remaining path. The response was `stopped-state trajectory accepted`. The plot shows the previously planned path, the executed portion, the braking segment, and the new path from the stopped state to the original target.

<p align="center">
  <a href="media/obstacle-replanning.png"><img src="media/obstacle-replanning.png" width="78%" alt="Seed 260928. A moved sphere obstructs the nominal path and the replanned tail reaches the same target."></a>
</p>

Call `RunOnlineReplanner()` for a new random scene and event. To choose the event type explicitly, pass a seed and one of the following strings, for example `RunOnlineReplanner(260928, 'orientation')`.

| Mode | Change at the event |
| --- | --- |
| `'position'` | Shift the measured tip position by 0.05–0.20 mm along the local tangent, in either direction. |
| `'orientation'` | Perturb the measured axial roll by 0.01–0.05° in either direction. |
| `'obstacle'` | Move a sphere into the future portion of the nominal trajectory. |

The demonstration prepares 201 candidate anchors along the nominal trajectory. [ReplanLibrary.m](Replan/ReplanLibrary.m) solves the remaining-horizon problems, and [ReplanSensitivity.m](Replan/ReplanSensitivity.m) checks which active-set Karush–Kuhn–Tucker systems can supply a local prediction. The console reports how many anchors are eligible. Library preparation takes place before the event. The paper uses a separate fixed protocol for its event timing measurements.

For a measured state deviation, [ReplanPredict.m](Replan/ReplanPredict.m) uses the nearest anchor and its stored factors to correct the remaining motion. If the corrected trajectory passes validation, it can be returned immediately. When braking is required, [ReplanEvent.m](Replan/ReplanEvent.m) obtains a stopped state and starts the constrained solve there. Inspect `out.reply.action` and `out.reply.result.success` to see which path the code took and whether it returned a trajectory.

## Inspect a run

| Output | Where to look |
| --- | --- |
| Offline candidates | `result.candidates` contains solver status, motion costs, and available acceptance diagnostics. If every candidate fails, the driver saves the result in `results/<case_id>.mat` before raising `NeedleOffline:NoAcceptedPlan`. |
| Position and controls | The nine columns of `reshape(result.best.z(2:end), result.d.n, 9)` are position (x, y, z), speed, acceleration, axial rate, azimuth, elevation, and axial roll. [PlotSolution.m](PlotSolution.m) converts the internal units for display. |
| Single-event output | The directory printed in `out.folder` contains `offline.mat`, `sensitivity_library.mat`, `online.mat`, and the trajectory and state/control plots in PNG and FIG formats. `out.eligible_count` and `out.anchor_count` report library coverage for that scene. |

If MATLAB reports `NeedleOffline:Dependency`, check that `fullfile(getenv('CASADI_ROOT'), '+casadi', 'SX.m')` exists. If CasADi loads but IPOPT cannot initialize MA97, check that your IPOPT installation has access to the HSL solver. The code requests MA97 explicitly in both entry points.

## How the MATLAB functions fit together

The entry points share the needle model and the offline planner. `RunMe` loads a numbered case and returns the selected motion. `RunOnlineReplanner` first computes that same kind of offline solution for a generated scene, then uses its states and controls to build the anchor library. The functions below follow the order in which a trajectory passes through the code.

### From a grid route to a timed needle motion

1. **Generate the route pool.** [`FindRoutes(d,c)`](FindRoutes.m) takes the scene geometry and search settings. It marks grid cells intersecting a sphere enlarged by the needle radius, constructs a 26-neighbor graph, and penalizes interior cells of previously returned paths. It removes duplicate grid paths. [`ResampleRoute(x,n)`](ResampleRoute.m) interpolates each polyline at equal arc-length intervals, so two routes can be compared at corresponding positions even if they pass through different numbers of cells. Starting with the shortest route, the selector repeatedly takes the route farthest from its nearest representative in root-mean-square point distance. In `result.routes`, `paths` holds the unique routes, `selected` indexes the representatives, and `cluster_distance_mm` gives the distance from each route to its nearest representative.
2. **Fit and optimize each candidate.** [`OfflinePlan(d,c)`](OfflinePlan.m) builds an initial position, attitude, speed, and control history from each representative. Its local `Candidate` routine fits a needle motion to the route before the terminally constrained solve. The direct seed starts from the entry-to-target line. The single-route seed uses the first graph path, and the LIOM seed uses successive corridor-penalty solves. [`BuildNLP`](BuildNLP.m) assembles the CasADi nonlinear program in scaled free variables and supplies the map back to the complete trajectory. [`NeedleStep(x,u,h,d)`](NeedleStep.m) advances the seven states by an explicit midpoint step while holding acceleration and axial rate constant on the interval. The first element of `z` is the interval length in seconds.
3. **Decide whether a candidate can enter the pool.** [`CheckTrajectory`](CheckTrajectory.m) checks node dynamics, boundary conditions, and bounds. Its `'posterior'` mode evaluates the centerline segments against the physical spheres, replays the controls using fourth-order Runge–Kutta integration, and checks nonadjacent body segments outside the arc-length exclusion neighborhood. The `Refine` routine in `OfflinePlan.m` can increase nearby planning radii and repeat the solve after a clearance failure. `audit_history` records the margins and outcome of each attempt.
4. **Choose from the accepted motions.** [`RiskField(z,d,c)`](RiskField.m) evaluates proximity at segment endpoints and midpoints and integrates it along the path. For spheres representing the same object it uses the nearest surface, then sums contributions from distinct objects. `integral_mm` records the integrated exposure, and `penalty_s` is its contribution to the score. The planner sorts accepted candidates by this score and breaks a tie using motion cost. `result.min_cost` and `result.best` give access to both choices.

To trace a particular solution, the following code starts with the result saved by `RunMe(1,false)`. The identifier of the selected candidate indexes the same cell array used for the acceptance count.

```matlab
load(fullfile('results', '1.mat'), 'result')
accepted = nnz(cellfun(@(s) s.success, result.candidates));
counts = [result.routes.unique_count, numel(result.routes.selected), accepted];
fprintf('%d unique routes, %d representatives, %d accepted candidates\n', counts)
chosen = result.candidates{result.best_id};
disp(chosen.method)
disp([chosen.check.duration, chosen.cost, chosen.selection_score])
disp([chosen.audit.clearance_lower_mm, chosen.audit.terminal_mm])
```

In the case 1 example, 48 unique routes led to six representatives and nine accepted candidates, including the direct, LIOM, and single-route seeds. The first numeric row gives insertion time, motion cost, and selection score in seconds. The second gives the replay clearance estimate and terminal position error in millimeters.

### What an anchor stores

[`ReplanRandomScene`](Replan/ReplanRandomScene.m) generates an entry, a target, and four, six, or eight obstacles, and checks a witness motion before offline planning. Once a nominal trajectory is accepted, [`ReplanLibrary`](Replan/ReplanLibrary.m) takes a tail beginning at each anchor node, retains the executed prefix, and sets the measured start and original target for the remaining problem. [`ReplanLocalObstacles`](Replan/ReplanLocalObstacles.m) identifies the inflated spheres intersecting the tail's local box. This subset reduces the number of obstacle constraints in the local optimization. The physical replay check covers the full scene.

[`ReplanBuild`](Replan/ReplanBuild.m) parameterizes the remaining problem by the event state, preceding control, and sphere centers. [`ReplanSolve`](Replan/ReplanSolve.m) returns the fitted tail, the IPOPT status, and the primal and dual quantities needed by [`ReplanSensitivity`](Replan/ReplanSensitivity.m). The latter assembles a square Karush–Kuhn–Tucker system from equality constraints and active inequalities and bounds. It checks weak activity, the condition estimate, stationarity, and the linear-system residual. An eligible entry stores a trajectory sensitivity matrix and a multiplier sensitivity matrix. [`ReplanPrepareCorrector`](Replan/ReplanPrepareCorrector.m) also factors the active-set matrix for residual correction.

[`ReplanSaveLibrary`](Replan/ReplanSaveLibrary.m) writes the numerical anchor records to `sensitivity_library.mat`. Each entry's `time`, `n`, `valid`, and `reason` identify its time, remaining node count, and eligibility decision. The live solver and corrector objects are used in the running MATLAB session. The saved numerical library can be inspected later:

```matlab
saved = load(fullfile(out.folder, 'sensitivity_library.mat'), 'lib');
[~, j] = min(abs(cellfun(@(e) e.time, saved.lib.entries) - out.event.time));
anchor = saved.lib.entries{j};
disp([anchor.time, anchor.n, anchor.valid])
disp(anchor.reason)
```

For the obstacle event at 6.633 s in the example above, the nearest anchor is at 6.583 s and has 269 remaining nodes. It passed the matrix checks. The changed obstacle geometry determines the response to this particular event, so the solver uses the stopped-state branch.

### What happens at the event

[`ReplanRandomEvent`](Replan/ReplanRandomEvent.m) returns the event node and time, measured start, updated sphere centers, and executed centerline. [`ReplanEvent`](Replan/ReplanEvent.m) dispatches that single measurement. The most direct way to follow a run is to look up the text returned in `out.reply.action`:

| Returned action | How the code reaches it |
| --- | --- |
| `unchanged reference accepted` | A sphere has moved. The original remaining trajectory passes both [discrete validation](Replan/ReplanValidate.m) against the measured start and [post-solve checks](Replan/ReplanAudit.m) on the updated geometry. The driver returns that tail directly. |
| `sensitivity accepted` | [`ReplanReplan`](Replan/ReplanReplan.m) locates the nearest anchor in nominal time. With unchanged sphere centers, an eligible anchor, and a perturbation within the screening threshold, [`ReplanPredict`](Replan/ReplanPredict.m) applies the stored sensitivities and can take up to three residual-correction steps. The trajectory must pass the planning-margin, physical-constraint, and replay checks. |
| `stopped-state trajectory accepted` | [`ReplanStopState`](Replan/ReplanStopState.m) integrates braking and checks the swept motion. A new query starts at the stopped state. For an obstacle intrusion, [`ReplanRepairSeed`](Replan/ReplanRepairSeed.m) adjusts intersecting interior seed points and restores the boundary values. [`ReplanSafeSolve`](Replan/ReplanSafeSolve.m) solves the fixed-mesh remaining problem and checks the resulting trajectory. |
| `braking validation rejected` or `hold: replanning validation rejected` | The first action identifies an inadmissible braking motion. The second means the stopped-state query did not return an accepted tail. In either case, `out.reply.result.success` is false. When the measured boundary itself fails validation, [`ReplanFailedResult`](Replan/ReplanFailedResult.m) supplies infeasible diagnostics for that query. |

[`ReplanBoundaryValid`](Replan/ReplanBoundaryValid.m) checks the event boundary against state limits and obstacle clearances. [`ReplanValidate`](Replan/ReplanValidate.m) evaluates scaled discrete residuals. [`ReplanAudit`](Replan/ReplanAudit.m) calls [`ReplanReplay`](Replan/ReplanReplay.m) to integrate the controls from the initial state of the candidate and [`ReplanSelfCheck`](Replan/ReplanSelfCheck.m) to check nonlocal separation along the joined prefix and remaining centerline. The returned `check` and `audit` structures expose these two sets of results separately.

### Drawing the motion

[`PlotSolution`](PlotSolution.m) and [`DrawScene`](DrawScene.m) share the physical geometry used by the planner. The nine state and control traces are plotted against insertion time. [`MakeVideo`](MakeVideo.m) reads an accepted result by case number or takes the result structure directly. [`NeedleFrame`](NeedleFrame.m) constructs the tip frame from azimuth, elevation, and axial roll, with its local z-axis along the needle tangent. For an event, [`ReplanPlot`](Replan/ReplanPlot.m) draws the executed motion, any braking segment, and the accepted tail together and places their state histories on the same time axis.
