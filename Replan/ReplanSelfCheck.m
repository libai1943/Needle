function s = ReplanSelfCheck(x, d, c)
    x = x(:, 1:3) * 1000 / d.scale;
    v = diff(x);
    len = sqrt(sum(v .^ 2, 2));
    arc = [0; cumsum(len)];
    mid = (x(1:end - 1, :) + x(2:end, :)) / 2;
    radius = 2 * d.needle_radius * 1000 / d.scale;

    if isfield(c, 'body_projection_certificate') && c.body_projection_certificate
        direction = x(end, :) - x(1, :);
        norm_direction = norm(direction);
        positive = len > 1e-12;
        if norm_direction > 1e-12 && any(positive)
            rho = min((v(positive, :) * (direction / norm_direction)') ./ len(positive));
            lower = rho * c.self_arc_mm;
            if lower >= radius + 1e-9
                s = struct('feasible', true, 'lower_bound_mm', lower, 'tested_pairs', 0, 'arc_exclusion_mm', c.self_arc_mm, 'method', 'projection_certificate');
                return
            end
        end
    end
    D = sqrt(max(0, sum(mid .^ 2, 2) + sum(mid .^ 2, 2)' - 2 * (mid * mid'))) - len / 2 - len' / 2;
    gap = arc(1:end - 1)' - arc(2:end);
    mask = gap >= c.self_arc_mm;
    D(~mask) = Inf;
    [ii, jj] = find(D < radius);
    actual = Inf;

    for k = 1:numel(ii)
        dist = segdist(x(ii(k), :), x(ii(k) + 1, :), x(jj(k), :), x(jj(k) + 1, :));
        actual = min(actual, dist);
        D(ii(k), jj(k)) = dist;
    end
    s = struct('feasible', actual >= radius, 'lower_bound_mm', min(D(:)), 'tested_pairs', numel(ii), 'arc_exclusion_mm', c.self_arc_mm, 'method', 'segment_pairs');
end

function d = segdist(p1, p2, q1, q2)
    u = p2 - p1;
    v = q2 - q1;
    w = p1 - q1;
    a = dot(u, u);
    b = dot(u, v);
    cc = dot(v, v);
    dd = dot(u, w);
    e = dot(v, w);
    points = [0, max(0, min(1, e / max(cc, eps))); 1, max(0, min(1, (e + b) / max(cc, eps))); max(0, min(1, -dd / max(a, eps))), 0; max(0, min(1, (b - dd) / max(a, eps))), 1];
    den = a * cc - b * b;

    if den > 1e-18
        t = (b * e - cc * dd) / den;
        s = (a * e - b * dd) / den;
        if t >= 0 && t <= 1 && s >= 0 && s <= 1
            points(end + 1, :) = [t, s];
        end
    end
    delta = w + points(:, 1) .* u - points(:, 2) .* v;
    d = sqrt(min(sum(delta .^ 2, 2)));
end
