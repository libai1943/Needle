function h = DrawScene(ax, result)
    d = result.d;
    q = reshape(result.best.z(2:end), d.n, 9);
    mm = 1000 / d.scale;
    path = q(:, 1:3) * mm;
    hold(ax, 'on');
    grid(ax, 'on');
    box(ax, 'on');
    set(ax, 'Color', 'w', 'FontName', 'Arial', 'FontSize', 11, 'GridAlpha', .13, 'LineWidth', .8);
    [sx, sy, sz] = sphere(28);

    for j = 1:numel(d.radii)
        r = d.radii(j) * mm;
        ctr = d.centers(:, j) * mm;
        surf(ax, sx * r + ctr(1), sy * r + ctr(2), sz * r + ctr(3), 'FaceColor', [.55, .64, .72], 'FaceAlpha', .23, 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'HandleVisibility', 'off');
    end
    h.reference = plot3(ax, path(:, 1), path(:, 2), path(:, 3), 'Color', [.72, .77, .80], 'LineWidth', 1.5);
    h.entry = plot3(ax, path(1, 1), path(1, 2), path(1, 3), 'o', 'MarkerFaceColor', [.13, .34, .62], 'MarkerEdgeColor', 'w', 'MarkerSize', 9, 'LineWidth', 1.1);
    h.target = plot3(ax, d.goal(1) * mm, d.goal(2) * mm, d.goal(3) * mm, 'p', 'MarkerFaceColor', [.87, .47, .16], 'MarkerEdgeColor', 'w', 'MarkerSize', 16, 'LineWidth', 1.1);
    points = [path; (d.centers - d.radii)' * mm; (d.centers + d.radii)' * mm];
    lo = min(points, [], 1) - 5;
    hi = max(points, [], 1) + 5;
    xlim(ax, [lo(1), hi(1)]);
    ylim(ax, [lo(2), hi(2)]);
    zlim(ax, [lo(3), hi(3)]);
    xlabel(ax, 'x (mm)');
    ylabel(ax, 'y (mm)');
    zlabel(ax, 'z (mm)');
    daspect(ax, [1, 1, 1]);
    view(ax, -58, 23);
    camproj(ax, 'orthographic');
    set(ax, 'CameraViewAngleMode', 'auto');
    h.light = camlight(ax, 'headlight');
end
