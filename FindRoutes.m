function r = FindRoutes(d, c)
    clock = tic;
    dim = ceil((d.ub - d.lb)' / d.grid);
    map = zeros(dim);
    axes_values = cell(1, 3);

    for k = 1:3
        axes_values{k} = d.lb(k) + d.grid / 2 + (0:dim(k) - 1) * d.grid;
    end
    for j = 1:numel(d.radii)
        radius = d.radii(j) + d.needle_radius;
        ctr = d.centers(:, j)';
        first = max(1, floor((ctr - radius - d.lb') / d.grid) + 1);
        last = min(dim, ceil((ctr + radius - d.lb') / d.grid));
        for ix = first(1):last(1)
            for iy = first(2):last(2)
                for iz = first(3):last(3)
                    lo = [axes_values{1}(ix), axes_values{2}(iy), axes_values{3}(iz)] - d.grid / 2;
                    hi = lo + d.grid;
                    distance = 0;
                    for axis = 1:3
                        if ctr(axis) < lo(axis)
                            distance = distance + (ctr(axis) - lo(axis)) ^ 2;
                        elseif ctr(axis) > hi(axis)
                            distance = distance + (ctr(axis) - hi(axis)) ^ 2;
                        end
                    end
                    if distance <= radius ^ 2
                        map(ix, iy, iz) = -1;
                    end
                end
            end
        end
    end
    start = max(1, min(dim, floor((d.start' - d.lb') / d.grid) + 1));
    goal = max(1, min(dim, floor((d.goal' - d.lb') / d.grid) + 1));
    start_id = sub2ind(dim, start(1), start(2), start(3));
    goal_id = sub2ind(dim, goal(1), goal(2), goal(3));
    count = prod(dim);
    [ix, iy, iz] = ind2sub(dim, (1:count)');
    xyz = [ix, iy, iz];
    [dx, dy, dz] = ndgrid(-1:1, -1:1, -1:1);
    offsets = [dx(:), dy(:), dz(:)];
    offsets(all(offsets == 0, 2), :) = [];
    source = cell(26, 1);
    target = source;
    free = find(map(:) ~= -1);

    for k = 1:26
        other = xyz(free, :) + offsets(k, :);
        valid = all(other >= 1 & other <= dim, 2);
        a = free(valid);
        b = other(valid, :);
        dest = b(:, 1) + (b(:, 2) - 1) * dim(1) + (b(:, 3) - 1) * dim(1) * dim(2);
        valid = map(dest) ~= -1;
        source{k} = a(valid);
        target{k} = dest(valid);
    end
    graph = digraph(vertcat(source{:}), vertcat(target{:}), [], count);
    ends = graph.Edges.EndNodes;
    destinations = ends(:, 2);
    distance = d.grid * sqrt(sum((xyz(ends(:, 1), :) - xyz(ends(:, 2), :)) .^ 2, 2));
    heuristic = d.grid * sqrt(sum((xyz - goal) .^ 2, 2));
    base_weight = distance + heuristic(ends(:, 2)) - heuristic(ends(:, 1));
    r = struct('setup_seconds', toc(clock), 'paths', {{}}, 'raw_count', 0, 'duplicates', 0, 'search_seconds', [], 'grid_costs', []);
    keys = containers.Map('KeyType', 'char', 'ValueType', 'logical');

    for k = 1:c.routes
        t = tic;
        graph.Edges.Weight = max(0, base_weight + map(destinations));
        if map(start_id) == -1 || map(goal_id) == -1
            path = [];
        else
            path = shortestpath(graph, start_id, goal_id, 'Method', 'positive');
        end
        [ix, iy, iz] = ind2sub(dim, path(:));
        sub = [ix, iy, iz];
        r.search_seconds(end + 1) = toc(t);
        r.raw_count = k;
        if isempty(sub)
            break
        end
        ids = sub2ind(dim, sub(:, 1), sub(:, 2), sub(:, 3));
        r.grid_costs(k) = d.grid * sum(sqrt(sum(diff(sub) .^ 2, 2))) + sum(map(ids));
        key = sprintf('%d,', sub');
        if isKey(keys, key)
            r.duplicates = r.duplicates + 1;
        else
            keys(key) = true;
            xyz_path = d.lb' + d.grid / 2 + (sub - 1) * d.grid;
            r.paths{end + 1} = ResampleRoute([d.start'; xyz_path; d.goal'], d.n);
        end
        for j = 2:size(sub, 1) - 1
            map(ids(j)) = map(ids(j)) + c.search_penalty;
        end
    end
    r.enumeration_seconds = toc(clock);
    t = tic;
    count = numel(r.paths);
    r.selected = [];
    r.cluster = [];

    if count > 0
        desc = zeros(count, 3 * c.descriptor_points);
        lens = zeros(count, 1);
        for j = 1:count
            xyz_path = ResampleRoute(r.paths{j}, c.descriptor_points);
            desc(j, :) = xyz_path(:)' * 1000 / d.scale;
            lens(j) = sum(sqrt(sum(diff(r.paths{j}) .^ 2, 2)));
        end
        [~, first] = min(lens);
        r.selected = first;
        distances = inf(count, 1);
        for j = 1:min(c.representatives, count)
            latest = r.selected(end);
            delta = desc - desc(latest, :);
            dist = sqrt(sum(delta .^ 2, 2) / c.descriptor_points);
            distances = min(distances, dist);
            distances(r.selected) = -Inf;
            [gap, idx] = max(distances);
            if gap < c.diversity_mm || j == min(c.representatives, count)
                break
            end
            r.selected(end + 1) = idx;
        end
        all_dist = zeros(count, numel(r.selected));
        for j = 1:numel(r.selected)
            all_dist(:, j) = sqrt(sum((desc - desc(r.selected(j), :)) .^ 2, 2) / c.descriptor_points);
        end
        [r.cluster_distance_mm, r.cluster] = min(all_dist, [], 2);
        r.lengths = lens;
        r.descriptors = desc;
    end
    r.selection_seconds = toc(t);
    r.unique_count = count;
end
