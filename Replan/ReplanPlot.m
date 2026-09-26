function figures = ReplanPlot(output)
    o = output.offline;
    d = o.d;
    mm = 1000 / d.scale;
    q = reshape(o.best.z(2:end), d.n, 9);
    event = output.event;
    reply = output.reply;
    r = reply.result;
    figures = gobjects(1, 2);
    figures(1) = figure('Color', 'w', 'Name', 'Single-event replanning', 'Position', [60, 60, 1200, 850]);
    ax = axes(figures(1), 'Position', [.09, .12, .84, .77]);
    h = DrawScene(ax, o);
    set(h.reference, 'Visible', 'off');
    nominal = plot3(ax, q(:, 1) * mm, q(:, 2) * mm, q(:, 3) * mm, '--', 'Color', [.45, .5, .56], 'LineWidth', 1.6);
    p = event.executed_prefix * mm;
    executed = plot3(ax, p(:, 1), p(:, 2), p(:, 3), 'Color', [.14, .23, .36], 'LineWidth', 3);
    marker = scatter3(ax, event.start(1) * mm, event.start(2) * mm, event.start(3) * mm, 75, [.9, .4, .08], 'filled');
    handles = [nominal, executed, marker, h.target];
    labels = {'Offline plan', 'Executed prefix', 'Event measurement', 'Target'};

    if strcmp(event.kind, 'obstacle')
        j = event.obstacle_id;
        [sx, sy, sz] = sphere(40);
        center = event.centers(:, j) * mm;
        radius = d.radii(j) * mm;
        surf(ax, center(1) + radius * sx, center(2) + radius * sy, center(3) + radius * sz, 'FaceColor', [.92, .3, .13], 'FaceAlpha', .35, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        old = event.old_center * mm;
        plot3(ax, [old(1), center(1)], [old(2), center(2)], [old(3), center(3)], ':', 'Color', [.92, .3, .13], 'LineWidth', 2, 'HandleVisibility', 'off');
    end
    if ~isempty(reply.brake)
        p = reply.brake.position_mm;
        handles(end + 1) = plot3(ax, p(:, 1), p(:, 2), p(:, 3), 'Color', [.9, .4, .08], 'LineWidth', 3);
        labels{end + 1} = 'Braking segment';
    end
    if r.success
        v = reshape(r.z(2:end), [], 9);
        handles(end + 1) = plot3(ax, v(:, 1) * mm, v(:, 2) * mm, v(:, 3) * mm, 'Color', [0, .55, .43], 'LineWidth', 3);
        labels{end + 1} = 'Accepted remaining trajectory';
    end
    points = [q(:, 1:3); (event.centers - d.radii)'; (event.centers + d.radii)'];

    if r.success
        points = [points; v(:, 1:3)];
    end
    limits = [min(points, [], 1) * mm - 5; max(points, [], 1) * mm + 5];
    xlim(ax, limits(:, 1)');
    ylim(ax, limits(:, 2)');
    zlim(ax, limits(:, 3)');
    legend(ax, handles, labels, 'Location', 'northoutside', 'NumColumns', 3, 'Box', 'off');
    title(ax, sprintf('%s event at %.2f s | %s', event.kind, event.time, reply.action), 'FontSize', 15);
    figures(2) = figure('Color', 'w', 'Name', 'Replanned states and controls', 'Position', [100, 90, 1250, 830]);
    tiles = tiledlayout(figures(2), 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    indices = [1, 2, 3, 7, 8, 9, 4, 5, 6];
    factors = [mm, mm, mm, 180 / pi, 180 / pi, 180 / pi, mm, mm, 180 / pi];
    names = {'x (mm)', 'y (mm)', 'z (mm)', 'Azimuth (deg)', 'Elevation (deg)', 'Axial roll (deg)', 'Speed (mm/s)', 'Acceleration (mm/s^2)', 'Axial rate (deg/s)'};
    time = (0:d.n - 1)' * o.best.z(1);

    for j = 1:9
        ax = nexttile(tiles);
        plot(ax, time, q(:, indices(j)) * factors(j), '--', 'Color', [.45, .5, .56], 'LineWidth', 1.3, 'DisplayName', 'Offline');
        hold(ax, 'on');
        if ~isempty(reply.brake) && isfield(reply.brake, 'state_path')
            xb = reply.brake.state_path;
            qb = zeros(size(xb, 1), 9);
            qb(:, [1:4, 7:9]) = xb;
            qb(:, 5) = -d.amax;
            tb = event.time + linspace(0, reply.brake.duration, size(qb, 1))';
            plot(ax, tb, qb(:, indices(j)) * factors(j), 'Color', [.9, .4, .08], 'LineWidth', 1.5, 'DisplayName', 'Braking');
        end
        if r.success
            tv = r.event.time + (0:size(v, 1) - 1)' * r.z(1);
            if j >= 8
                stairs(ax, tv, v(:, indices(j)) * factors(j), 'Color', [0, .55, .43], 'LineWidth', 1.8, 'DisplayName', 'Replanned');
            else
                plot(ax, tv, v(:, indices(j)) * factors(j), 'Color', [0, .55, .43], 'LineWidth', 1.8, 'DisplayName', 'Replanned');
            end
        end
        xline(ax, event.time, ':', 'Color', [.9, .4, .08], 'LineWidth', 1.3, 'HandleVisibility', 'off');
        xlabel(ax, 'Motion time (s)');
        ylabel(ax, names{j});
        grid(ax, 'on');
        set(ax, 'FontName', 'Arial', 'FontSize', 10);
        if j == 1
            legend(ax, 'Location', 'best', 'Box', 'off');
        end
    end
    title(tiles, sprintf('Random scene %d | %s', output.seed, reply.action));
    drawnow;
    names = {'online_trajectory', 'online_variables'};

    for j = 1:2
        file = fullfile(output.folder, names{j});
        exportgraphics(figures(j), [file, '.png'], 'Resolution', 220);
        savefig(figures(j), [file, '.fig']);
    end
end
