function d = ReplanRandomScene(d, c)
    n = c.n;
    d.n = n;
    d.case_id = 0;
    d.grid = c.grid_mm * d.scale / 1000;
    d.start = d.scale / 1000 * [24 * rand - 12; 24 * rand - 12; 0];
    h = (26 + 10 * rand) / (n - 1);
    q = zeros(n, 9);
    q(1, 1:3) = d.start';
    q(1, 7:9) = [-pi + 2 * pi * rand, (43 + 20 * rand) * pi / 180, pi / 2];
    q(2:end, 4) = d.vmax * sin(linspace(0, pi, n - 1)') .^ 2;
    q(end, 4) = 0;
    q(1:end - 1, 5) = diff(q(:, 4)) / h;
    ids = [1:4, 7:9];

    for k = 1:n - 1
        q(k + 1, ids) = NeedleStep(q(k, ids), q(k, 5:6), h, d);
    end
    d.goal = q(end, 1:3)';
    count = 4 + 2 * randi([0, 2]);
    d.object_ids = 1:count;
    d.centers = zeros(3, count);
    d.radii = zeros(1, count);

    for j = 1:count
        placed = false;
        for attempt = 1:500
            k = round(n * (.2 + .6 * rand));
            direction = randn(3, 1);
            direction = direction / norm(direction);
            radius = (2.5 + 4.5 * rand) * d.scale / 1000;
            margin = (.8 + 3.2 * rand) * d.scale / 1000;
            center = q(k, 1:3)' + (radius + d.needle_radius + margin) * direction;
            distance = sqrt(sum((q(:, 1:3) - center') .^ 2, 2));
            endpoints = min(norm(center - d.start), norm(center - d.goal));
            if all(center >= d.lb - radius) && all(center <= d.ub + radius) && min(distance) > radius + d.needle_radius + .4 * d.scale / 1000 && endpoints > radius + d.needle_radius + 9 * d.scale / 1000
                placed = true;
                break
            end
        end
        assert(placed, 'Could not generate the random obstacle geometry.');
        d.centers(:, j) = center;
        d.radii(j) = radius;
    end
    witness = [h; q(:)];
    nodal = CheckTrajectory(witness, d, c, 'offline');
    posterior = CheckTrajectory(witness, d, c, 'posterior');
    assert(nodal.feasible && posterior.feasible, 'NeedleOffline:Scene', 'The random scene failed its independent feasibility precheck.');
end
