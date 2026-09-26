function ok = ReplanBoundaryValid(d, p)
    q = p(1:9);
    lo = [d.lb; 0; -d.amax; -d.wmax; -20; -20; -20];
    hi = [d.ub; d.vmax; d.amax; d.wmax; 20; 20; 20];
    ok = all(isfinite(p)) && all(q >= lo - 1e-10) && all(q <= hi + 1e-10) && all(d.goal >= d.lb) && all(d.goal <= d.ub) && cos(q(8)) ^ 2 >= d.cos_limit ^ 2;
    ctr = reshape(p(10:end), 3, []);
    radius = d.radii + d.needle_radius;
    ok = ok && all(sqrt(sum((ctr - q(1:3)) .^ 2, 1)) >= radius - 1e-10) && all(sqrt(sum((ctr - d.goal) .^ 2, 1)) >= radius - 1e-10);
end
