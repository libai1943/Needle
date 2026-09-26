function z = ReplanRepairSeed(z, d, p, box)
    q = reshape(z(2:end), d.n, 9);
    centers = reshape(p(10:end), 3, []);
    margin = 1e-5 * d.scale;

    for iter = 1:12
        changed = false;
        for j = 1:size(centers, 2)
            delta = q(:, 1:3) - centers(:, j)';
            dist = sqrt(sum(delta .^ 2, 2));
            rad = d.radii(j) + d.needle_radius + margin;
            bad = find(dist < rad);
            bad = bad(bad > 1 & bad < d.n);
            for k = bad'
                dir = delta(k, :);
                if norm(dir) < eps
                    dir = [1, 0, 0];
                end
                q(k, 1:3) = centers(:, j)' + rad * dir / norm(dir);
                changed = true;
            end
        end
        q(2:end - 1, 1:3) = max(box(1, :), min(box(2, :), q(2:end - 1, 1:3)));
        if ~changed
            break
        end
    end
    state_ids = [1:4, 7:9];
    q(1, state_ids) = p(state_ids)';
    q(end, 1:3) = d.goal';
    q(end, 4:6) = 0;
    z = [max(d.tmin / (d.n - 1), min(d.tmax / (d.n - 1), z(1))); q(:)];
end
