function [z, info] = ReplanPredict(e, p, c)
    delta = p - e.p0;
    changed = find(delta ~= 0);
    z = e.sol + e.matrix(:, changed) * delta(changed);
    u = z(e.model.free) ./ e.model.scale(e.model.free);
    lambda = e.base_multipliers + e.multiplier_matrix(:, changed) * delta(changed);
    t = e.corrector;
    res = full(t.residual(u, p, lambda));
    info = struct('steps', 0, 'initial_residual', norm(res, Inf), 'final_residual', norm(res, Inf), 'valid', false);

    for k = 1:c.corrector_steps
        candidate = full(e.model.expand(u, p));
        v = ReplanValidate(candidate, e.planning_d, c, 'online', p, e.extra);
        if v.feasible
            break
        end
        rhs = -res;
        duall = zeros(size(rhs));
        duall(t.Q) = t.U \ (t.L \ rhs(t.P));
        unew = u + duall(1:numel(u));
        lnew = lambda + duall(numel(u) + 1:end);
        rnew = full(t.residual(unew, p, lnew));
        if any(~isfinite(rnew)) || norm(rnew, Inf) > 4 * max(1e-8, norm(res, Inf))
            break
        end
        u = unew;
        lambda = lnew;
        res = rnew;
        info.steps = k;
    end
    z = full(e.model.expand(u, p));
    info.final_residual = norm(res, Inf);
    info.valid = all(isfinite(z));
end
