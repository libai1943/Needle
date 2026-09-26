function lib = ReplanLibrary(o, c)
    assert(o.success, 'Offline planning must produce a validated nominal.');
    t = tic;
    d = o.d;
    q = reshape(o.best.z(2:end), d.n, 9);
    h = o.best.z(1);
    last = max(1, d.n - min(c.min_tail_nodes, d.n) + 1);
    ids = unique(round(linspace(1, last, c.anchors)));
    lib.entries = cell(1, numel(ids));
    lib.nominal = o.best.z;
    lib.d = d;
    lib.config = c;

    for j = 1:numel(ids)
        k = ids(j);
        tail = q(k:end, :);
        dt = d;
        dt.n = size(tail, 1);
        dt.tmin = .01;
        dt.executed_prefix = q(1:k - 1, 1:3);
        box = [max(d.lb', min(tail(:, 1:3)) - c.local_box_mm * d.scale / 1000); min(d.ub', max(tail(:, 1:3)) + c.local_box_mm * d.scale / 1000)];
        concerned = ReplanLocalObstacles(d.centers, d.radii + d.needle_radius, box);
        extra = struct('box', box, 'obstacle_ids', concerned, 'initial_margin_mm', o.best.margin_mm);
        [s, b] = ReplanSafeSolve(dt, c, 'online', [h; tail(:)], [tail(1, :)'; dt.centers(:)], extra);
        e = ReplanSensitivity(b, s);
        e.time = (k - 1) * h;
        e.node = k;
        e.d = dt;
        e.extra = b.extra;
        e.solve = s;
        e.n = dt.n;
        e.planning_d = b.d;
        e.model = struct('free', b.free, 'scale', b.scale, 'expand', b.expand);
        lib.entries{j} = e;
        if mod(j, 25) == 0 || j == numel(ids)
            fprintf('Sensitivity library: %d/%d anchors, %d eligible.\n', j, numel(ids), sum(cellfun(@(v) v.valid, lib.entries(1:j))));
        end
    end
    lib.seconds = toc(t);
end
