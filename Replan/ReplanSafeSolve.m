function [s, b] = ReplanSafeSolve(d, c, mode, z0, p, extra, cached)
    if nargin < 6
        extra = struct();
    end
    if nargin < 7
        cached = [];
    end
    if isempty(p)
        p = [zeros(9, 1); d.centers(:)];
    end
    clock = tic;
    margins = zeros(size(d.radii));
    history = {};
    physical = d;
    physical.centers = reshape(p(10:end), 3, []);

    if isfield(extra, 'initial_margin_mm')
        margins = reshape(extra.initial_margin_mm, size(d.radii));
        assert(all(margins >= 0));
    end
    for pass = 1:c.inflation_passes
        dd = physical;
        dd.radii = d.radii + margins * d.scale / 1000;
        current_extra = extra;
        if isfield(extra, 'box') && isfield(extra, 'obstacle_ids')
            current_extra.obstacle_ids = ReplanLocalObstacles(dd.centers, dd.radii + dd.needle_radius, extra.box);
        end
        reuse = ~isempty(cached) && isequal(dd.radii, cached.d.radii) && isequal(dd.goal, cached.d.goal) && isequal(dd.lb, cached.d.lb) && isequal(dd.ub, cached.d.ub) && c.max_cpu == cached.c.max_cpu && isequal(current_extra.box, cached.extra.box) && isequal(current_extra.obstacle_ids, cached.extra.obstacle_ids);
        if reuse
            b = cached;
        else
            b = ReplanBuild(dd, c, mode, reshape(z0(2:end), d.n, 9), current_extra);
        end
        s = ReplanSolve(b, z0, p);
        rec = struct('status', s.status, 'margins_mm', margins, 'seconds', s.seconds, 'nlp_feasible', s.success, 'model_reused', reuse);
        if ~s.success
            history{end + 1} = rec;
            break
        end
        a = ReplanAudit(s.z, physical, c, p);
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
    s.check = ReplanValidate(s.z, physical, c, mode, p, extra);
    s.success = s.success && s.check.feasible;
end
