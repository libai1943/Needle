function t = ReplanPrepareCorrector(b, e)
    import casadi.*
    x = b.symbolic;
    constraint = [x.eq; x.ineq(e.active_inequalities); b.lb(e.active_lower) - x.u(e.active_lower); x.u(e.active_upper) - b.ub(e.active_upper)];
    constraint = constraint(e.retained_constraint_rows);
    lambda = SX.sym('lambda', numel(constraint));
    residual = [gradient(x.f + dot(lambda, constraint), x.u); constraint];
    t.residual = Function('active_kkt_residual', {x.u, x.p, lambda}, {residual});
    [t.L, t.U, t.P, t.Q] = lu(e.kkt_matrix, 'vector');
    t.inequality_multiplier_rows = find(e.retained_constraint_rows > e.equality_count);
end
