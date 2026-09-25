function r = RiskField(z, d, c)
    centers = d.centers;
    q = reshape(z(2:end), d.n, 9);
    x = q(:, 1:3) * 1000 / d.scale;
    ctr = centers * 1000 / d.scale;
    rad = (d.radii + d.needle_radius) * 1000 / d.scale;
    mid = (x(1:end - 1, :) + x(2:end, :)) / 2;
    lengths = sqrt(sum(diff(x) .^ 2, 2));
    phi = zeros(d.n, 1);
    phi_mid = zeros(d.n - 1, 1);
    groups = 1:numel(rad);

    if isfield(d, 'object_ids')
        groups = d.object_ids;
    end
    for g = unique(groups)
        gap = inf(d.n, 1);
        mid_gap = inf(d.n - 1, 1);
        for j = find(groups == g)
            gap = min(gap, sqrt(sum((x - ctr(:, j)') .^ 2, 2)) - rad(j));
            mid_gap = min(mid_gap, sqrt(sum((mid - ctr(:, j)') .^ 2, 2)) - rad(j));
        end
        phi = phi + max(0, 1 - gap / c.risk_distance_mm) .^ 2;
        phi_mid = phi_mid + max(0, 1 - mid_gap / c.risk_distance_mm) .^ 2;
    end
    increment = lengths .* (phi(1:end - 1) + 4 * phi_mid + phi(2:end)) / 6;
    integral_mm = sum(increment);
    r = struct('integral_mm', integral_mm, 'penalty_s', c.risk_weight * integral_mm / (d.vmax * 1000 / d.scale), 'field_at_nodes', phi, 'arc_mm', [0; cumsum(lengths)], 'cumulative_integral_mm', [0; cumsum(increment)], 'distance_mm', c.risk_distance_mm, 'weight', c.risk_weight);
end
