function r = ReplanReplan(lib, event, options)
    if nargin < 3
        options = struct();
    end
    if ~isfield(options, 'predict')
        options.predict = true;
    end
    if ~isfield(options, 'force_nlp')
        options.force_nlp = false;
    end
    if ~isfield(options, 'repair')
        options.repair = true;
    end
    if ~isfield(options, 'local')
        options.local = true;
    end
    if ~isfield(options, 'fast_only')
        options.fast_only = false;
    end
    clock = tic;
    c = lib.config;
    [~, idx] = min(abs(cellfun(@(e) e.time, lib.entries) - event.time));
    e = lib.entries{idx};
    d = e.d;
    d.centers = event.centers;

    if isfield(event, 'executed_prefix')
        d.executed_prefix = event.executed_prefix;
    end
    assert(~isfield(event, 'goal'), 'This version has a fixed target.');
    p = [event.start(:); event.centers(:)];
    delta = p - e.p0;
    ps = [.01 * d.scale * ones(3, 1); d.vmax; d.amax; d.wmax; ones(3, 1); .01 * d.scale * ones(numel(d.centers), 1)];
    local = norm(delta ./ ps, Inf) <= c.trust_limit;
    obstacle_change = norm(event.centers - lib.d.centers, 'fro') > 0;
    r = struct('success', false, 'method', 'rejected', 'anchor', idx, 'anchor_time', e.time, 'event_time', event.time, 'mapping_ms', 0, 'validation_ms', 0, 'solve_ms', 0, 'build_ms', 0, 'total_ms', 0, 'fallback', false);

    if ~ReplanBoundaryValid(d, p)
        failed = ReplanFailedResult(d, c, 'Boundary_invalid');
        r.status = 'Boundary_invalid';
        r.z = e.sol;
        r.check = failed.check;
        r.audit = [];
        r.obstacles_total = numel(d.radii);
        r.obstacles_kept = numel(d.radii);
        r.tail_nodes = d.n;
        r.p = p;
        r.event = event;
        r.total_ms = toc(clock) * 1000;
        return
    end
    t = tic;
    z = e.sol;
    r.corrector = struct('steps', 0, 'initial_residual', NaN, 'final_residual', NaN, 'valid', false);

    if options.predict && e.valid && ~obstacle_change && local
        [z, r.corrector] = ReplanPredict(e, p, c);
    end
    q = reshape(z(2:end), d.n, 9);
    ids = [1:4, 7:9];
    q(1, ids) = event.start(ids)';
    q(end, 1:3) = d.goal';
    q(end, 4:6) = 0;
    z = [z(1); q(:)];
    r.mapping_ms = toc(t) * 1000;
    t = tic;
    check = ReplanValidate(z, d, c, 'online', p, e.extra);
    planningcheck = ReplanValidate(z, e.planning_d, c, 'online', p, e.extra);
    audit = [];

    if check.feasible
        audit = ReplanAudit(z, d, c, p);
    end
    r.anchor_eligible = e.valid;
    r.within_trust_region = local;
    r.planning_check = planningcheck;
    accepted = options.predict && e.valid && r.corrector.valid && ~obstacle_change && local && check.feasible && planningcheck.feasible && ~isempty(audit) && audit.feasible;
    r.validation_ms = toc(t) * 1000;
    extra = e.extra;

    if accepted && ~options.force_nlp
        r.success = true;
        r.method = 'sensitivity';
        r.z = z;
        r.check = check;
        r.audit = audit;
        r.margin_mm = e.solve.margin_mm;
    else
        if options.fast_only
            r.z = z;
            r.check = check;
            r.audit = audit;
            r.p = p;
            r.event = event;
            r.total_ms = toc(clock) * 1000;
            return
        end
        r.fallback = true;
        cc = c;
        cc.max_cpu = c.online_cpu;
        if options.local
            box = e.extra.box;
            box(1, :) = max(d.lb', min([box(1, :); event.start(1:3)'; d.goal']));
            box(2, :) = min(d.ub', max([box(2, :); event.start(1:3)'; d.goal']));
            extra.box = box;
            extra.obstacle_ids = ReplanLocalObstacles(d.centers, d.radii + d.needle_radius, box);
        else
            extra = struct();
            box = [d.lb'; d.ub'];
        end
        extra.initial_margin_mm = e.solve.margin_mm;
        if options.repair
            z = ReplanRepairSeed(z, d, p, box);
        end
        cached = [];
        if options.local && isfield(e, 'fallback_model')
            cached = e.fallback_model;
        end
        t = tic;
        [s, b] = ReplanSafeSolve(d, cc, 'online', z, p, extra, cached);
        r.solve_ms = toc(t) * 1000;
        extra = b.extra;
        r.z = s.z;
        r.check = s.check;
        r.method = 'warm_nlp';
        r.success = s.success;
        r.status = s.status;
        r.margin_mm = s.margin_mm;
        r.safety_passes = s.safety_passes;
        if isfield(s, 'audit')
            r.audit = s.audit;
        end
        r.safety_history = s.audit_history;
        r.model_reused = any(cellfun(@(a) a.model_reused, s.audit_history));
    end
    r.obstacles_total = numel(d.radii);
    r.obstacles_kept = numel(d.radii);

    if isfield(extra, 'obstacle_ids')
        r.obstacles_kept = numel(extra.obstacle_ids);
    end
    r.tail_nodes = d.n;
    r.p = p;
    r.event = event;
    r.total_ms = toc(clock) * 1000;
end
