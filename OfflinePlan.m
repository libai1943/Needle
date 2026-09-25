function o = OfflinePlan(d, c)
    clock = tic;
    r = FindRoutes(d, c);
    jobs = r.paths(r.selected);

    if isempty(jobs)
        jobs = {ResampleRoute([d.start'; d.goal'], d.n)};
    end
    all = cell(1, numel(jobs));

    for k = 1:numel(jobs)
        all{k} = Candidate(d, c, jobs{k}, 'Representative');
        fprintf('Case %d, representative %d/%d: %s, J %.6f, %.2f s\n', d.case_id, k, numel(jobs), all{k}.status, all{k}.cost, all{k}.total_seconds);
    end
    all{end + 1} = Candidate(d, c, ResampleRoute([d.start'; d.goal'], d.n), 'Direct');

    if ~isempty(r.paths)
        all{end + 1} = Candidate(d, c, r.paths{1}, 'LIOM');
        all{end + 1} = Candidate(d, c, r.paths{1}, 'Single A*');
    end
    for k = 1:numel(all)
        if all{k}.success
            all{k}.risk = RiskField(all{k}.z, d, c);
            all{k}.selection_score = all{k}.cost + all{k}.risk.penalty_s;
        end
    end
    o = struct('d', d, 'config', c, 'routes', r, 'candidates', {all}, 'success', false, 'best', [], 'min_cost', [], 'seconds', toc(clock));
    valid = find(cellfun(@(s) s.success, all));

    if ~isempty(valid)
        costs = cellfun(@(s) s.cost, all(valid));
        [~, idx] = min(costs);
        o.min_cost = all{valid(idx)};
        scores = cellfun(@(s) [s.selection_score, s.cost], all(valid), 'UniformOutput', false);
        [~, order] = sortrows(vertcat(scores{:}), [1, 2]);
        o.best_id = valid(order(1));
        o.best = all{o.best_id};
        o.success = true;
    end
    o.seconds = toc(clock);
end

function s = Candidate(d, c, route, method)
    clock = tic;
    z0 = InitialGuess(d, route);
    stages = {};

    if strcmp(method, 'LIOM')
        for k = 1:4
            q = reshape(z0(2:end), d.n, 9);
            box = Corridor(q(:, 1:3), d);
            if isempty(box)
                break
            end
            extra = struct('corridor', box, 'penalty', 10 ^ k);
            b = BuildNLP(d, c, 'offline', [], extra);
            st = SolveNLP(b, z0);
            stages{end + 1} = struct('status', st.status, 'seconds', st.seconds, 'eq', st.check.eq);
            z0 = st.z;
        end
    elseif ~strcmp(method, 'Direct')
        b = BuildNLP(d, c, 'presolve', route);
        st = SolveNLP(b, z0);
        stages{end + 1} = struct('status', st.status, 'seconds', st.seconds, 'eq', st.check.eq);
        if all(isfinite(st.z))
            z0 = st.z;
        end
    end
    s = Refine(d, c, z0);
    s.total_seconds = toc(clock);
    s.method = method;
    s.stages = stages;
end

function s = Refine(d, c, z0)
    clock = tic;
    margins = zeros(size(d.radii));
    history = {};

    for pass = 1:c.inflation_passes
        dd = d;
        dd.radii = d.radii + margins * d.scale / 1000;
        b = BuildNLP(dd, c, 'offline', reshape(z0(2:end), d.n, 9));
        s = SolveNLP(b, z0);
        rec = struct('status', s.status, 'margins_mm', margins, 'seconds', s.seconds, 'nlp_feasible', s.success);
        if ~s.success
            history{end + 1} = rec;
            break
        end
        a = CheckTrajectory(s.z, d, c, 'posterior');
        rec.audit = rmfield(a, 'position_mm');
        history{end + 1} = rec;
        s.audit = a;
        if a.feasible
            break
        end
        s.success = false;
        if ~a.self.feasible
            s.status = 'Self_contact_rejected';
            break
        end
        if ~a.replay_bounds_ok || a.terminal_mm > c.audit_target_mm
            s.status = 'Replay_accuracy_rejected';
            break
        end
        near = a.replay_by_obstacle_mm < c.audit_near_mm;
        step = c.inflation_step_mm + max(0, -min(a.clearance_lower_by_obstacle_mm));
        margins(near) = margins(near) + step;
        z0 = s.z;
        s.status = 'Posterior_collision_rejected';
    end
    s.audit_history = history;
    s.margin_mm = margins;
    s.safety_seconds = toc(clock);
    s.safety_passes = numel(history);
    s.check = CheckTrajectory(s.z, d, c, 'offline');
    s.success = s.success && s.check.feasible;
end

function s = SolveNLP(b, z0)
    t = tic;
    u0 = z0(b.free) ./ b.scale(b.free);
    r = b.solver('x0', u0, 'p', b.p0, 'lbx', b.lb, 'ubx', b.ub, 'lbg', b.lbg, 'ubg', b.ubg);
    stats = b.solver.stats();
    s.z = full(b.expand(full(r.x), b.p0));
    s.cost = full(r.f);
    s.status = stats.return_status;
    s.iterations = stats.iter_count;
    s.seconds = toc(t);

    if strcmp(s.status, 'Invalid_Option')
        error('NeedleOffline:Solver', 'IPOPT could not initialize its configured MA97 solver. Check the CasADi and HSL installation.');
    end
    s.check = CheckTrajectory(s.z, b.d, b.c, b.mode, b.extra);
    s.success = any(strcmp(s.status, {'Solve_Succeeded', 'Solved_To_Acceptable_Level'})) && s.check.feasible;
end

function z = InitialGuess(d, route)
    n = d.n;
    q = zeros(n, 9);
    q(:, 1:3) = route;
    delta = diff(route);
    delta = [delta; delta(end, :)];
    q(:, 7) = unwrap(atan2(delta(:, 2), delta(:, 1)));
    q(:, 8) = atan2(delta(:, 3), sqrt(sum(delta(:, 1:2) .^ 2, 2)));
    q(:, 8) = min(acos(d.cos_limit) - .01, max(0, q(:, 8)));
    h = d.tmax / (n - 1);
    q(:, 4) = min(d.vmax, sqrt(sum(delta .^ 2, 2)) / h);
    q([1, end], 4:6) = 0;
    z = [h; q(:)];
end

function boxes = Corridor(route, d)
    boxes = zeros(size(route, 1), 6);
    step = .002 * d.scale;
    centers = d.centers';
    radii = d.radii' + d.needle_radius;

    for i = 1:size(route, 1)
        lo = route(i, :);
        hi = lo;
        if any(sqrt(sum((centers - lo) .^ 2, 2)) <= radii)
            boxes = [];
            return
        end
        active = true(1, 6);
        for iter = 1:80
            if ~any(active)
                break
            end
            for face = find(active)
                a = lo;
                b = hi;
                ax = mod(face - 1, 3) + 1;
                if face <= 3
                    a(ax) = max(d.lb(ax), a(ax) - step);
                else
                    b(ax) = min(d.ub(ax), b(ax) + step);
                end
                nearest = max(a, min(b, centers));
                if any(sum((centers - nearest) .^ 2, 2) <= radii .^ 2) || isequal([a, b], [lo, hi])
                    active(face) = false;
                else
                    lo = a;
                    hi = b;
                end
            end
        end
        boxes(i, :) = [lo, hi];
    end
end
