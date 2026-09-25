function [result, figures] = RunMe(case_id, make_video)
    if nargin < 1
        case_id = 1;
    end
    if nargin < 2
        make_video = false;
    end
    validateattributes(case_id, {'numeric'}, {'scalar', 'integer', '>=', 1, '<=', 20}, mfilename, 'case_id');
    validateattributes(make_video, {'logical', 'numeric'}, {'scalar', 'binary'}, mfilename, 'make_video');
    root = fileparts(mfilename('fullpath'));

    if exist('casadi.SX', 'class') ~= 8
        casadi_root = getenv('CASADI_ROOT');
        if isempty(casadi_root) && isfolder(fullfile(root, 'casadi'))
            casadi_root = fullfile(root, 'casadi');
        end
        if isempty(casadi_root) && isfolder('C:/casadi')
            casadi_root = 'C:/casadi';
        end
        assert(isfile(fullfile(casadi_root, '+casadi', 'SX.m')), 'NeedleOffline:Dependency', 'Install CasADi 3.6.1 with IPOPT/MA97 and set CASADI_ROOT to its MATLAB directory.');
        addpath(casadi_root);
    end
    names = {'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS'};
    values = cellfun(@getenv, names, 'UniformOutput', false);

    for k = 1:numel(names)
        setenv(names{k}, '1');
    end
    threads = maxNumCompThreads(1);
    runtime_guard = onCleanup(@() RestoreRuntime(names, values, threads));
    c = struct('n', 400, 'routes', 48, 'representatives', 6, 'descriptor_points', 40, 'diversity_mm', 2, 'grid_mm', 5, 'search_penalty', .1);
    c.max_iter = 1600;
    c.max_cpu = 12;
    c.linear_solver = 'ma97';
    c.effort_weight = .02;
    c.smooth_weight = .002;
    c.attitude_weight = .001;
    c.eq_tol = 1e-6;
    c.ineq_tol = 1e-7;
    c.risk_distance_mm = 5;
    c.risk_weight = 1;
    c.audit_substeps = 8;
    c.audit_target_mm = .15;
    c.audit_padding_mm = .002;
    c.inflation_step_mm = .05;
    c.inflation_passes = 6;
    c.self_arc_mm = 4;
    c.audit_near_mm = 5;
    bank = load(fullfile(root, 'Cases.mat'), 'cases');
    d = bank.cases{case_id};
    assert(d.n == c.n && d.grid == c.grid_mm * d.scale / 1000 && d.case_id == case_id, 'NeedleOffline:Data', 'The case geometry and configuration do not match.');
    fprintf('Offline case %d/20: %d spheres, %d nodes, %d searches, up to %d representatives.\n', case_id, numel(d.radii), d.n, c.routes, c.representatives);
    result = OfflinePlan(d, c);
    out = fullfile(root, 'results');

    if ~isfolder(out)
        mkdir(out);
    end
    save(fullfile(out, sprintf('%d.mat', case_id)), 'result', '-v7');
    assert(result.success, 'NeedleOffline:NoAcceptedPlan', 'No trajectory passed the fixed-budget solve and validation. Candidate diagnostics were saved in results/%d.mat.', case_id);
    figures = PlotSolution(result, out);
    fprintf('Accepted: J %.6f s, S %.6f s, insertion %.3f s, replay clearance %.6f mm, planning %.2f s.\n', result.best.cost, result.best.selection_score, result.best.check.duration, result.best.audit.clearance_lower_mm, result.seconds);

    if make_video
        MakeVideo(result);
    end
end

function RestoreRuntime(names, values, threads)
    maxNumCompThreads(threads);

    for k = 1:numel(names)
        setenv(names{k}, values{k});
    end
end
