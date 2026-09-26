function [output, figures] = RunOnlineReplanner(seed, event_mode)
    if nargin < 1 || isempty(seed)
        rng('shuffle');
        seed = randi(2 ^ 31 - 1);
    end
    if nargin < 2
        event_mode = 'random';
    end
    validateattributes(seed, {'numeric'}, {'scalar', 'integer', '>=', 0, '<', 2 ^ 32});
    event_mode = validatestring(event_mode, {'random', 'position', 'orientation', 'obstacle'});
    root = fileparts(mfilename('fullpath'));
    addpath(fullfile(root, 'Replan'));

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
    c.anchors = 201;
    c.min_tail_nodes = 50;
    c.local_box_mm = 15;
    c.trust_limit = .15;
    c.online_cpu = 30;
    c.active_tol = 2e-6;
    c.dual_tol = 1e-7;
    c.sensitivity_cond_max = 1e13;
    c.corrector_steps = 3;
    c.body_projection_certificate = true;
    c.check_segments = false;
    rng(seed, 'twister');
    bank = load(fullfile(root, 'Cases.mat'), 'cases');
    d = ReplanRandomScene(bank.cases{1}, c);
    folder = fullfile(root, 'results', sprintf('online_%d_%s', seed, datestr(now, 'yyyymmdd_HHMMSS')));
    mkdir(folder);
    fprintf('Random scene %d: offline planning, %d spheres.\n', seed, numel(d.radii));
    offline = OfflinePlan(d, c);
    save(fullfile(folder, 'offline.mat'), 'offline', 'seed', '-v7');
    assert(offline.success, 'NeedleOffline:RandomPlan', 'No offline candidate passed validation. Diagnostics: %s', folder);
    library = ReplanLibrary(offline, c);
    ReplanSaveLibrary(fullfile(folder, 'sensitivity_library.mat'), library);
    rng(mod(seed + 1, 2 ^ 32), 'twister');
    event = ReplanRandomEvent(library, event_mode);
    reply = ReplanEvent(library, event);
    output = struct('seed', seed, 'offline', offline, 'event', event, 'reply', reply, 'folder', folder, 'anchor_count', numel(library.entries), 'eligible_count', sum(cellfun(@(e) e.valid, library.entries)), 'library_seconds', library.seconds);
    save(fullfile(folder, 'online.mat'), 'output', '-v7');
    figures = ReplanPlot(output);
    fprintf('%s at %.3f s: %s. Results: %s\n', event.kind, event.time, reply.action, folder);
end

function RestoreRuntime(names, values, threads)
    maxNumCompThreads(threads);

    for k = 1:numel(names)
        setenv(names{k}, values{k});
    end
end
