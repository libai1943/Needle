function file = MakeVideo(source, output_dir)
    root = fileparts(mfilename('fullpath'));
    if isnumeric(source)
        validateattributes(source, {'numeric'}, {'scalar', 'integer', '>=', 1, '<=', 20}, mfilename, 'case_id');
        input = fullfile(root, 'results', sprintf('%d.mat', source));
        assert(isfile(input), 'NeedleOffline:Video', 'Run RunMe(%d) before generating this video.', source);
        saved = load(input, 'result');
        result = saved.result;
    else
        result = source;
    end
    assert(isstruct(result) && isfield(result, 'success') && result.success, 'NeedleOffline:Video', 'An accepted offline result is required.');
    if nargin < 2
        output_dir = fullfile(root, 'videos');
    end
    if ~isfolder(output_dir)
        mkdir(output_dir);
    end
    d = result.d;
    s = result.best;
    q = reshape(s.z(2:end), d.n, 9);
    t = (0:d.n - 1)' * s.z(1);
    mm = 1000 / d.scale;
    path = q(:, 1:3) * mm;
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [20, 20, 1920, 1080], 'Renderer', 'opengl', 'GraphicsSmoothing', 'on', 'MenuBar', 'none', 'ToolBar', 'none');
    figure_guard = onCleanup(@() CloseFigure(fig));
    set(fig, 'PaperUnits', 'inches', 'PaperPosition', [0, 0, 16, 9], 'PaperSize', [16, 9], 'InvertHardcopy', 'off');
    ax = axes(fig, 'Position', [.045, .235, .61, .59]);
    h = DrawScene(ax, result);
    set(ax, 'FontSize', 14);
    xlim(ax, xlim(ax) + [-5, 5]);
    ylim(ax, ylim(ax) + [-5, 5]);
    zlim(ax, zlim(ax) + [-5, 5]);
    body = plot3(ax, path(1, 1), path(1, 2), path(1, 3), 'Color', [0, .48, .43], 'LineWidth', 4.5);
    tip = plot3(ax, path(1, 1), path(1, 2), path(1, 3), 'o', 'MarkerFaceColor', [0, .48, .43], 'MarkerEdgeColor', 'w', 'MarkerSize', 9, 'LineWidth', 1.3);
    tip_frame = CreateFrame(ax, 6, 13);
    key = legend(ax, [h.reference, body, h.entry, h.target], {'Planned path', 'Inserted centerline', 'Entry', 'Target'}, 'Orientation', 'horizontal', 'Box', 'off', 'FontSize', 11, 'AutoUpdate', 'off');
    set(key, 'Units', 'normalized', 'Position', [.045, .115, .61, .035]);
    annotation(fig, 'textbox', [.04, .925, .92, .06], 'String', sprintf('FLEXIBLE NEEDLE PLANNING  |  CASE %d', d.case_id), 'EdgeColor', 'none', 'FontName', 'Arial', 'FontSize', 26, 'FontWeight', 'bold', 'Color', [.10, .19, .27]);
    phase = annotation(fig, 'textbox', [.045, .855, .62, .045], 'String', '', 'EdgeColor', 'none', 'FontName', 'Arial', 'FontSize', 16, 'FontWeight', 'bold', 'Color', [0, .45, .40]);
    annotation(fig, 'textbox', [.045, .03, .60, .055], 'String', sprintf('%d spheres  |  T %.2f s  |  J %.3f s  |  S %.3f s\nReplay clearance %.4f mm', numel(d.radii), t(end), s.cost, s.selection_score, s.audit.clearance_lower_mm), 'EdgeColor', 'none', 'FontName', 'Arial', 'FontSize', 13, 'Color', [.25, .31, .36], 'HorizontalAlignment', 'center');
    annotation(fig, 'line', [.69, .69], [.07, .91], 'Color', [.85, .88, .90], 'LineWidth', 1);
    annotation(fig, 'textbox', [.715, .865, .27, .045], 'String', 'NEEDLE-TIP LOCAL FRAME', 'EdgeColor', 'none', 'FontName', 'Arial', 'FontSize', 17, 'FontWeight', 'bold', 'Color', [.10, .19, .27]);
    annotation(fig, 'textbox', [.715, .825, .27, .035], 'String', 'Fixed world view  |  z_n: insertion axis', 'Interpreter', 'none', 'EdgeColor', 'none', 'FontName', 'Arial', 'FontSize', 12, 'Color', [.35, .41, .46]);
    local_ax = axes(fig, 'Position', [.745, .565, .215, .255]);
    hold(local_ax, 'on');
    axis(local_ax, 'equal');
    axis(local_ax, [-1.6, 1.6, -1.6, 1.6, -1.6, 1.6]);
    axis(local_ax, 'off');
    initial_rotation = NeedleFrame(q(1, 7), q(1, 8), q(1, 9));
    campos(local_ax, (initial_rotation * [3; -4; 2.6])');
    camtarget(local_ax, [0, 0, 0]);
    camup(local_ax, [0, 0, 1]);
    camproj(local_ax, 'orthographic');
    camva(local_ax, 30);
    world_labels = {'X_w', 'Y_w', 'Z_w'};
    for j = 1:3
        ref = zeros(3, 2);
        ref(j, :) = [-1.4, 1.4];
        plot3(local_ax, ref(1, :), ref(2, :), ref(3, :), ':', 'Color', [.69, .73, .77], 'LineWidth', 1.0);
        text(local_ax, ref(1, 2), ref(2, 2), ref(3, 2), world_labels{j}, 'Color', [.48, .52, .56], 'FontSize', 10, 'FontName', 'Arial');
    end
    shaft = plot3(local_ax, [0, 0], [0, 0], [0, 0], 'Color', [.37, .43, .47], 'LineWidth', 9);
    plane = patch(local_ax, 'XData', zeros(1, 4), 'YData', zeros(1, 4), 'ZData', zeros(1, 4), 'FaceColor', [.90, .52, .25], 'FaceAlpha', .12, 'EdgeColor', [.80, .59, .44], 'LineWidth', .7);
    local_frame = CreateFrame(local_ax, 1, 15);
    plot3(local_ax, 0, 0, 0, 'o', 'MarkerFaceColor', [.12, .19, .24], 'MarkerEdgeColor', 'w', 'MarkerSize', 7);
    attitude = annotation(fig, 'textbox', [.715, .535, .27, .04], 'String', '', 'EdgeColor', 'none', 'FontName', 'Arial', 'FontSize', 13, 'HorizontalAlignment', 'center', 'Color', [.25, .31, .36]);
    highlights = gobjects(1, 3);
    cursors = gobjects(1, 3);
    dots = gobjects(1, 3);
    values = [q(:, 4) * mm, q(:, 9) * 180 / pi, q(:, 6) * 180 / pi];
    labels = {'Speed (mm/s)', 'Axial roll (deg)', 'Axial rate (deg/s)'};

    for j = 1:3
        axes_j = axes(fig, 'Position', [.76, .39 - (j - 1) * .155, .215, .10]);
        hold(axes_j, 'on');
        if j < 3
            plot(axes_j, t, values(:, j), 'Color', [.76, .80, .83], 'LineWidth', 1.4);
            highlights(j) = plot(axes_j, t(1), values(1, j), 'Color', [0, .48, .43], 'LineWidth', 2.2);
        else
            stairs(axes_j, t, values(:, j), 'Color', [.76, .80, .83], 'LineWidth', 1.4);
            highlights(j) = stairs(axes_j, t(1), values(1, j), 'Color', [0, .48, .43], 'LineWidth', 2.2);
        end
        dots(j) = plot(axes_j, t(1), values(1, j), 'o', 'MarkerFaceColor', [.87, .47, .16], 'MarkerEdgeColor', 'none', 'MarkerSize', 5);
        cursors(j) = xline(axes_j, 0, ':', 'Color', [.57, .63, .67]);
        span = max(values(:, j)) - min(values(:, j));
        pad = max(.1, .12 * span);
        ylim(axes_j, [min(values(:, j)) - pad, max(values(:, j)) + pad]);
        xlim(axes_j, [0, t(end)]);
        grid(axes_j, 'on');
        box(axes_j, 'on');
        ylabel(axes_j, labels{j});
        xlabel(axes_j, 'Time (s)');
        set(axes_j, 'FontName', 'Arial', 'FontSize', 11, 'GridAlpha', .13);
    end
    file = fullfile(output_dir, sprintf('%d.mp4', d.case_id));
    writer = VideoWriter(file, 'MPEG-4');
    writer.Quality = 95;
    writer.FrameRate = 30;
    open(writer);
    writer_guard = onCleanup(@() CloseWriter(writer));
    frames = 720;
    capture_mode = 'native';

    for frame = 1:frames
        elapsed = (frame - 1) / writer.FrameRate;
        progress = min(1, max(0, (elapsed - 2) / 20));
        model_time = progress * t(end);
        node = min(d.n, max(1, floor(model_time / s.z(1)) + 1));
        if progress == 1
            node = d.n;
        end
        state = interp1(t, q(:, [1:4, 7:9]), model_time, 'linear');
        point = state(1:3) * mm;
        trace = [path(1:node, :); point];
        set(body, 'XData', trace(:, 1), 'YData', trace(:, 2), 'ZData', trace(:, 3));
        future = [point; path(min(node + 1, d.n):end, :)];
        set(h.reference, 'XData', future(:, 1), 'YData', future(:, 2), 'ZData', future(:, 3));
        set(tip, 'XData', point(1), 'YData', point(2), 'ZData', point(3));
        rotation = NeedleFrame(state(5), state(6), state(7));
        UpdateFrame(tip_frame, point, rotation);
        UpdateFrame(local_frame, [0, 0, 0], rotation);
        shaft_start = -1.25 * rotation(:, 3);
        set(shaft, 'XData', [shaft_start(1), 0], 'YData', [shaft_start(2), 0], 'ZData', [shaft_start(3), 0]);
        corners = rotation(:, 1:2) * [.43, -.43, -.43, .43; .43, .43, -.43, -.43];
        set(plane, 'XData', corners(1, :), 'YData', corners(2, :), 'ZData', corners(3, :));
        set(attitude, 'String', sprintf('Axial roll %.1f deg  |  Change %+.1f deg', state(7) * 180 / pi, (state(7) - q(1, 9)) * 180 / pi));
        view(ax, -58 + 110 * (frame - 1) / (frames - 1), 23 + 3 * sin(pi * (frame - 1) / (frames - 1)));
        camlight(h.light, 'headlight');
        if elapsed < 2
            label = 'Geometry and accepted offline trajectory';
        elseif progress < 1
            label = sprintf('Planned insertion  |  model time %.2f / %.2f s', model_time, t(end));
        else
            label = sprintf('Target reached  |  posterior checks passed  |  T %.2f s', t(end));
        end
        set(phase, 'String', label);
        for j = 1:3
            if j < 3
                current = interp1(t, values(:, j), model_time, 'linear');
            else
                current = values(node, j);
            end
            set(highlights(j), 'XData', [t(1:node); model_time], 'YData', [values(1:node, j); current]);
            set(dots(j), 'XData', model_time, 'YData', current);
            set(cursors(j), 'Value', model_time);
        end
        drawnow;
        [captured, capture_mode] = CaptureFrame(fig, capture_mode);
        assert(size(captured, 1) == 1080 && size(captured, 2) == 1920, 'NeedleOffline:VideoResolution', 'The graphics renderer must provide a full 1920-by-1080 frame.');
        writeVideo(writer, captured);
        if mod(frame, 60) == 0
            fprintf('Video case %d: %d/%d frames.\n', d.case_id, frame, frames);
        end
    end
    close(writer);
    clear writer_guard;
    close(fig);
    clear figure_guard;
    info = dir(file);
    assert(info.bytes < 200 * 1024 ^ 2, 'NeedleOffline:VideoSize', 'Video exceeds 200 MB.');
    fprintf('Saved %s (1920x1080, 30 fps, %.2f MB).\n', file, info.bytes / 1024 ^ 2);
end

function [pixels, mode] = CaptureFrame(fig, mode)
    if strcmp(mode, 'native')
        frame = getframe(fig);
        height = size(frame.cdata, 1);
        width = size(frame.cdata, 2);
        if width == 1920 && height >= 1026 && height <= 1080
            pixels = repmat(uint8(255), 1080, 1920, 3);
            first = floor((1080 - height) / 2) + 1;
            pixels(first:first + height - 1, :, :) = frame.cdata;
            return
        end
        mode = 'offscreen';
    end
    pixels = print(fig, '-RGBImage', '-r120');
end

function h = CreateFrame(ax, length_mm, font_size)
    colors = [.85, .16, .18; .15, .59, .27; .12, .36, .85];
    names = {'x_n', 'y_n', 'z_n'};
    h.arrows = gobjects(1, 3);
    h.labels = gobjects(1, 3);
    h.length = length_mm;
    for j = 1:3
        h.arrows(j) = quiver3(ax, 0, 0, 0, 0, 0, 0, 0, 'Color', colors(j, :), 'LineWidth', 2.5, 'MaxHeadSize', .55, 'HandleVisibility', 'off');
        h.labels(j) = text(ax, 0, 0, 0, names{j}, 'Color', colors(j, :), 'FontName', 'Arial', 'FontSize', font_size, 'FontWeight', 'bold', 'BackgroundColor', 'w', 'Margin', .4, 'Clipping', 'off', 'HorizontalAlignment', 'center', 'HandleVisibility', 'off');
    end
end

function UpdateFrame(h, point, rotation)
    for j = 1:3
        direction = h.length * rotation(:, j);
        set(h.arrows(j), 'XData', point(1), 'YData', point(2), 'ZData', point(3), 'UData', direction(1), 'VData', direction(2), 'WData', direction(3));
        set(h.labels(j), 'Position', point(:)' + 1.20 * direction');
    end
end

function CloseWriter(writer)
    try
        close(writer);
    catch
    end
end

function CloseFigure(fig)
    if isgraphics(fig)
        close(fig);
    end
end
