function b = BuildNLP(d, c, mode, reference, extra)
    if nargin < 5
        extra = struct();
    end
    import casadi.*
    n = d.n;
    np = 9 + numel(d.centers);
    p = SX.sym('p', np);
    qscale = [.1 * d.scale * ones(1, 3), d.vmax, d.amax, d.wmax, 1, 1, 1];
    zs = [d.tmax / (n - 1); kron(qscale', ones(n, 1))];
    low = repmat([d.lb', 0, -d.amax, -d.wmax, -20, -20, -20], n, 1);
    upp = repmat([d.ub', d.vmax, d.amax, d.wmax, 20, 20, 20], n, 1);
    low(1, 1:2) = d.start(1:2)' - d.start_slack;
    upp(1, 1:2) = d.start(1:2)' + d.start_slack;
    low(1, 7:9) = [-pi, 0, -pi];
    upp(1, 7:9) = [pi, acos(d.cos_limit), pi];
    fixed = [2 + 2 * n; 2 + n * (3:5)'];
    values = [d.start(3); zeros(3, 1)];

    if ~strcmp(mode, 'presolve')
        fixed = [fixed; 1 + n * (1:3)'];
        values = [values; d.goal];
    end
    fixed = [fixed; 1 + n * (4:6)'];
    values = [values; zeros(3, 1)];

    if isfield(extra, 'corridor')
        low(:, 1:3) = max(low(:, 1:3), extra.corridor(:, 1:3));
        upp(:, 1:3) = min(upp(:, 1:3), extra.corridor(:, 4:6));
    end
    lb = [d.tmin / (n - 1); low(:)];
    ub = [d.tmax / (n - 1); upp(:)];
    free = setdiff((1:1 + 9 * n)', fixed);
    u = SX.sym('u', numel(free));
    z = SX.zeros(1 + 9 * n, 1);
    z(free) = u .* zs(free);
    z(fixed) = values;
    h = z(1);
    q = reshape(z(2:end), n, 9);
    ids = [1:4, 7:9];
    res = q(2:end, ids) - NeedleStep(q(1:end - 1, ids), q(1:end - 1, 5:6), h, d);
    dyn = reshape(res(:, [1:3, 5, 6, 4, 7]), 7 * (n - 1), 1);
    eqscale = kron([.1 * d.scale * ones(3, 1); 1; 1; d.vmax; 1], ones(n - 1, 1));
    eq = dyn ./ eqscale;
    ineq = d.cos_limit ^ 2 - cos(q(:, 8)) .^ 2;
    ids = 1:size(d.centers, 2);

    for j = ids
        ctr = p(9 + (3 * j - 2:3 * j));
        r = d.radii(j) + d.needle_radius;
        ineq = [ineq; (r ^ 2 - sum((q(:, 1:3) - repmat(ctr', n, 1)) .^ 2, 2)) / max(r ^ 2, (.001 * d.scale) ^ 2)];
    end
    effort = h * (sumsqr(q(:, 5) / d.amax) + sumsqr(q(:, 6) / d.wmax));
    smooth = (sumsqr(diff(q(:, 5) / d.amax)) + sumsqr(diff(q(:, 6) / d.wmax))) / h;
    f = (n - 1) * h + c.effort_weight * effort + c.smooth_weight * smooth;
    gap = cos(q(:, 8)) .^ 2 - d.cos_limit ^ 2;
    positive_gap = .5 * (gap + sqrt(gap .^ 2 + 1e-12));
    f = f - c.attitude_weight * h * sum(log(positive_gap / (1 - d.cos_limit ^ 2)));

    if strcmp(mode, 'presolve')
        weights = linspace(1, 10, n)';
        f = sumsqr(repmat(weights, 1, 3) .* (q(:, 1:3) - reference) / (.1 * d.scale));
    end
    if isfield(extra, 'penalty')
        f = f + extra.penalty * sumsqr(eq);
        eq = SX.zeros(0, 1);
    end
    g = [eq; ineq];
    ne = numel(eq);
    opts = struct('print_time', false, 'error_on_fail', false);
    opts.ipopt = struct('print_level', 0, 'max_iter', c.max_iter, 'max_cpu_time', c.max_cpu, 'tol', 1e-9, 'acceptable_tol', 1e-7, 'constr_viol_tol', 1e-9, 'acceptable_constr_viol_tol', 1e-7, 'mu_strategy', 'adaptive', 'linear_solver', c.linear_solver, 'bound_relax_factor', 0, 'bound_push', 1e-8, 'bound_frac', 1e-8);
    b.solver = nlpsol('needle_nlp', 'ipopt', struct('x', u, 'p', p, 'f', f, 'g', g), opts);
    b.expand = Function('expand', {u, p}, {z});
    b.lb = lb(free) ./ zs(free);
    b.ub = ub(free) ./ zs(free);
    b.lbg = [zeros(ne, 1); -inf(numel(ineq), 1)];
    b.ubg = zeros(numel(g), 1);
    b.free = free;
    b.scale = zs;
    b.d = d;
    b.c = c;
    b.mode = mode;
    b.p0 = [zeros(9, 1); d.centers(:)];
    b.extra = extra;
end
