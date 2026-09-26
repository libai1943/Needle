function x = ReplanReplay(z, d, substeps)
    persistent cache

    if isempty(cache)
        cache = containers.Map();
    end
    key = sprintf('%d_%d_%.12g_%.12g', d.n, substeps, d.ka, d.kb);

    if ~isKey(cache, key)
        import casadi.*
        y = SX.sym('y', 7);
        u = SX.sym('u', 3);
        dt = u(3);
        k1 = rhs(y, u, d);
        k2 = rhs(y + dt * k1 / 2, u, d);
        k3 = rhs(y + dt * k2 / 2, u, d);
        k4 = rhs(y + dt * k3, u, d);
        one = Function('flow', {y, u}, {y + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6});
        cache(key) = one.mapaccum((d.n - 1) * substeps);
    end
    q = reshape(z(2:end), d.n, 9);
    u = repelem(q(1:end - 1, 5:6)', 1, substeps);
    u(3, :) = z(1) / substeps;
    f = cache(key);
    x = [q(1, [1:4, 7:9]); full(f(q(1, [1:4, 7:9])', u))'];
end

function f = rhs(x, u, d)
    k = d.ka - d.kb * u(2) ^ 2;
    v = x(4);
    a = x(5);
    b = x(6);
    g = x(7);
    f = [v * cos(a) * cos(b); v * sin(a) * cos(b); v * sin(b); u(1); k * v * sin(g) / cos(b); k * v * cos(g); u(2)];
end
