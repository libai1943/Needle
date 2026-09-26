function e = ReplanSensitivity(b, s)
    import casadi.*
    clock = tic;
    e = struct('valid', false, 'reason', '', 'matrix', [], 'sol', s.z, 'p0', s.p);
    e.condition = Inf;
    e.stationarity = Inf;
    e.dimension = NaN;
    e.check = s.check;
    e.status = s.status;

    if ~s.success
        e.reason = 'Anchor solve failed validation';
        return
    end
    x = b.symbolic;
    nu = numel(x.u);
    ne = numel(x.eq);
    [~, ~, gv] = b.eval(s.u, s.p);
    gv = full(gv);
    ai = find(gv >= -b.c.active_tol & s.lam_g(ne + 1:end) > b.c.dual_tol);
    al = find(s.u - b.lb <= b.c.active_tol & s.lam_x < -b.c.dual_tol);
    au = find(b.ub - s.u <= b.c.active_tol & s.lam_x > b.c.dual_tol);
    constraint = [x.eq; x.ineq(ai); b.lb(al) - x.u(al); x.u(au) - b.ub(au)];
    mult = [s.lam_g(1:ne); s.lam_g(ne + ai); -s.lam_x(al); s.lam_x(au)];
    lag = x.f + dot(mult, constraint);
    grad = gradient(lag, x.u);
    H = hessian(lag, x.u);
    A = jacobian(constraint, x.u);
    B = jacobian(constraint, x.p);
    Lp = jacobian(grad, x.p);
    fn = Function('kkt_blocks', {x.u, x.p}, {H, A, B, Lp, grad});
    [Hv, Av, Bv, Lv, station] = fn(s.u, s.p);
    Hv = sparse(Hv);
    Av = sparse(Av);
    Bv = full(Bv);
    Lv = full(Lv);
    keep = find(full(sum(abs(Av), 2)) > 1e-12);
    e.removed_constant_rows = size(Av, 1) - numel(keep);
    removed = setdiff((1:size(Av, 1))', keep);
    e.constant_parameter_dependence = 0;

    if ~isempty(removed)
        e.constant_parameter_dependence = norm(Bv(removed, :), Inf);
    end
    if ~isempty(removed) && (nnz(A(removed, :)) > 0 || nnz(B(removed, :)) > 0)
        e.reason = 'Numerically zero active row is not structurally constant';
        e.seconds = toc(clock);
        return
    end
    if e.constant_parameter_dependence > 1e-10
        e.reason = 'A constant active row changes with the parameter';
        e.seconds = toc(clock);
        return
    end
    Av = Av(keep, :);
    Bv = Bv(keep, :);
    M = [Hv, Av'; Av, sparse(size(Av, 1), size(Av, 1))];
    e.condition = condest(M);
    e.stationarity = norm(full(station), Inf);
    e.dimension = size(M, 1);
    e.active_inequalities = ai;
    e.active_lower = al;
    e.active_upper = au;
    e.weak_active = sum(gv >= -b.c.active_tol & s.lam_g(ne + 1:end) <= b.c.dual_tol) + sum(s.u - b.lb <= b.c.active_tol & s.lam_x >= -b.c.dual_tol) + sum(b.ub - s.u <= b.c.active_tol & s.lam_x <= b.c.dual_tol);
    e.rank_reduced = false;

    if e.weak_active > 0
        e.reason = 'Weakly active inequality or bound: no strict active-set predictor';
        e.seconds = toc(clock);
        return
    end
    if ~isfinite(e.condition) || e.condition > b.c.sensitivity_cond_max
        e.reason = 'Singular or ill-conditioned active KKT system';
        e.seconds = toc(clock);
        return
    end
    rhs = -[Lv; Bv];
    out = M \ rhs;
    du = out(1:nu, :);
    e.dimension = size(M, 1);
    e.linear_residual = norm(M * out - rhs, Inf) / max(1, norm(rhs, Inf));
    e.dependency_matrix = zeros(0, numel(s.p));
    e.dependency_residual = 0;
    e.minimum_active_multiplier = min([Inf; mult(ne + 1:end)]);
    e.maximum_inactive_inequality = max([-Inf; gv(setdiff(1:numel(gv), ai))]);
    e.parameter_columns = numel(s.p);
    e.free_variables = nu;
    e.retained_rows = size(Av, 1);
    expand = Function('expansion_derivative', {x.u, x.p}, {jacobian(x.z, x.u), jacobian(x.z, x.p)});
    [Zu, Zp] = expand(s.u, s.p);
    e.matrix = full(Zu) * du + full(Zp);
    obstacle_ids = 1:numel(b.d.radii);

    if isfield(b.extra, 'obstacle_ids')
        obstacle_ids = b.extra.obstacle_ids;
    end
    active_obstacles = [];

    for j = 1:numel(obstacle_ids)
        if any(ai > b.d.n * j & ai <= b.d.n * (j + 1))
            active_obstacles(end + 1) = obstacle_ids(j);
        end
    end
    inactive_obstacles = setdiff(1:numel(b.d.radii), active_obstacles);
    cols = [];

    for j = inactive_obstacles
        cols = [cols, 9 + (3 * j - 2:3 * j)];
    end
    e.max_inactive_obstacle_sensitivity = 0;

    if ~isempty(cols)
        e.max_inactive_obstacle_sensitivity = max(abs(e.matrix(:, cols)), [], 'all');
    end
    e.active_obstacle_count = numel(active_obstacles);
    e.multiplier_matrix = out(nu + 1:end, :);
    e.kkt_matrix = M;
    e.retained_constraint_rows = keep;
    e.base_multipliers = mult(keep);
    e.equality_count = ne;
    e.valid = all(isfinite(e.matrix(:))) && e.linear_residual < 1e-7 && e.stationarity < 1e-5;
    e.requires_direction_check = e.dependency_residual >= 1e-5;

    if e.valid
        e.reason = 'Square active-set KKT predictor passed numerical diagnostics';
    else
        e.reason = 'KKT residual or stationarity check failed';
    end
    if e.valid
        e.corrector = ReplanPrepareCorrector(b, e);
    end
    e.seconds = toc(clock);
end
