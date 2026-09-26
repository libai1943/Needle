function event = ReplanRandomEvent(lib, mode)
    d = lib.d;
    q = reshape(lib.nominal(2:end), d.n, 9);
    modes = {'position', 'orientation', 'obstacle'};

    if strcmp(mode, 'random')
        mode = modes{randi(numel(modes))};
    end
    last = min(floor(.75 * d.n), d.n - lib.config.min_tail_nodes + 1);

    if strcmp(mode, 'obstacle')
        last = floor(.5 * d.n);
    end
    k = randi([ceil(.2 * d.n), last]);
    event = struct('kind', mode, 'time', (k - 1) * lib.nominal(1), 'node', k, 'start', q(k, :)', 'nominal_start', q(k, :)', 'centers', d.centers, 'executed_prefix', q(1:k - 1, 1:3));

    if strcmp(mode, 'position')
        direction = [cos(q(k, 7)) * cos(q(k, 8)); sin(q(k, 7)) * cos(q(k, 8)); sin(q(k, 8))];
        event.start(1:3) = event.start(1:3) + (2 * (rand > .5) - 1) * (.05 + .15 * rand) * d.scale / 1000 * direction;
    elseif strcmp(mode, 'orientation')
        event.start(9) = event.start(9) + (2 * (rand > .5) - 1) * (.01 + .04 * rand) * pi / 180;
    else
        j = randi(numel(d.radii));
        future = randi([ceil(.68 * d.n), floor(.78 * d.n)]);
        direction = randn(3, 1);
        direction = direction / norm(direction);
        tangent = [cos(q(future, 7)) * cos(q(future, 8)); sin(q(future, 7)) * cos(q(future, 8)); sin(q(future, 8))];
        direction = direction - tangent * dot(tangent, direction);
        direction = direction / norm(direction);
        intrusion = (.2 + .6 * rand) * d.scale / 1000;
        event.centers(:, j) = q(future, 1:3)' + (d.radii(j) + d.needle_radius - intrusion) * direction;
        event.obstacle_id = j;
        event.old_center = d.centers(:, j);
        event.new_center = event.centers(:, j);
    end
end
