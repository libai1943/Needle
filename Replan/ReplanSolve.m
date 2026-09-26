function s = ReplanSolve(b, z0, p, warm)
    if nargin < 3
        p = b.p0;
    end
    if nargin < 4
        warm = [];
    end
    t = tic;
    u0 = z0(b.free) ./ b.scale(b.free);
    args = {'x0', u0, 'p', p, 'lbx', b.lb, 'ubx', b.ub, 'lbg', b.lbg, 'ubg', b.ubg};

    if ~isempty(warm)
        args = [args, {'lam_x0', warm.lam_x, 'lam_g0', warm.lam_g}];
    end
    r = b.solver(args{:});
    stats = b.solver.stats();
    s.u = full(r.x);
    s.z = full(b.expand(s.u, p));
    s.p = p;
    s.lam_x = full(r.lam_x);
    s.lam_g = full(r.lam_g);
    s.cost = full(r.f);
    s.status = stats.return_status;
    s.iterations = stats.iter_count;
    s.seconds = toc(t);
    s.check = ReplanValidate(s.z, b.d, b.c, b.mode, p, b.extra);
    s.success = any(strcmp(s.status, {'Solve_Succeeded', 'Solved_To_Acceptable_Level'})) && s.check.feasible;
end
