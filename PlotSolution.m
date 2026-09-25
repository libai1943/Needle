function figures = PlotSolution(result, output_dir)
    assert(result.success, 'NeedleOffline:Plot', 'An accepted trajectory is required.');
    d = result.d;
    s = result.best;
    q = reshape(s.z(2:end), d.n, 9);
    t = (0:d.n - 1)' * s.z(1);
    mm = 1000 / d.scale;
    figures = gobjects(1, 2);
    figures(1) = figure('Color', 'w', 'Name', sprintf('Case %d | offline trajectory', d.case_id), 'NumberTitle', 'off', 'Position', [80, 70, 1100, 790]);
    ax = axes(figures(1), 'Position', [.09, .19, .82, .67]);
    h = DrawScene(ax, result);
    set(h.reference, 'Visible', 'off');
    path = q(:, 1:3) * mm;
    line = plot3(ax, path(:, 1), path(:, 2), path(:, 3), 'Color', [0, .48, .43], 'LineWidth', 3);
    legend(ax, [line, h.entry, h.target], {'Selected trajectory', 'Entry', 'Target'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
    annotation(figures(1), 'textbox', [.07, .91, .90, .055], 'String', sprintf('CASE %d  |  %d obstacles  |  %d routes / %d representatives', d.case_id, numel(d.radii), result.routes.unique_count, numel(result.routes.selected)), 'EdgeColor', 'none', 'FontName', 'Arial', 'FontSize', 17, 'FontWeight', 'bold', 'Color', [.10, .19, .27]);
    annotation(figures(1), 'textbox', [.08, .025, .90, .075], 'String', sprintf('Insertion %.2f s    |    Cost J %.3f s    |    Score S %.3f s\nReplay clearance %.4f mm    |    Replay target error %.3g mm', s.check.duration, s.cost, s.selection_score, s.audit.clearance_lower_mm, s.audit.terminal_mm), 'EdgeColor', 'none', 'FontName', 'Arial', 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', [.22, .29, .34]);
    figures(2) = figure('Color', 'w', 'Name', sprintf('Case %d | states and controls', d.case_id), 'NumberTitle', 'off', 'Position', [120, 100, 1250, 840]);
    layout = tiledlayout(figures(2), 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    indices = [1, 2, 3, 7, 8, 9, 4, 5, 6];
    factors = [mm, mm, mm, 180 / pi, 180 / pi, 180 / pi, mm, mm, 180 / pi];
    labels = {'x (mm)', 'y (mm)', 'z (mm)', 'Azimuth \alpha (deg)', 'Elevation \beta (deg)', 'Axial roll \gamma (deg)', 'Insertion speed (mm/s)', 'Acceleration (mm/s^2)', 'Axial rate (deg/s)'};

    for k = 1:9
        ax = nexttile(layout);
        values = q(:, indices(k)) * factors(k);
        if k >= 8
            stairs(ax, t, values, 'Color', [.82, .40, .12], 'LineWidth', 1.8);
        else
            plot(ax, t, values, 'Color', [0, .48, .43], 'LineWidth', 1.8);
        end
        grid(ax, 'on');
        box(ax, 'on');
        xlim(ax, [0, t(end)]);
        xlabel(ax, 'Time (s)');
        ylabel(ax, labels{k});
        set(ax, 'FontName', 'Arial', 'FontSize', 10, 'GridAlpha', .15);
    end
    title(layout, sprintf('Case %d | Optimized states and controls | T = %.3f s', d.case_id, t(end)), 'FontName', 'Arial', 'FontSize', 16, 'Color', [.10, .19, .27]);
    drawnow;

    if nargin > 1
        if ~isfolder(output_dir)
            mkdir(output_dir);
        end
        names = {'trajectory', 'variables'};
        for k = 1:2
            file = fullfile(output_dir, sprintf('%d_%s', d.case_id, names{k}));
            set(figures(k), 'PaperPositionMode', 'auto', 'InvertHardcopy', 'off');
            print(figures(k), [file, '.png'], '-dpng', '-r160');
            savefig(figures(k), [file, '.fig']);
        end
    end
end
