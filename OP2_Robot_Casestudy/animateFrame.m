function [margin, com, poly] = animateFrame(ax, robot, info, q, ov)
%ANIMATEFRAME  Draw one frame of the combined demo and return the CoM stability margin.
%   Robot (Subtopic 1), CoM + trail + floor projection (Subtopic 2),
%   support polygon coloured by the margin (Subtopic 3), ZMP and capture point
%   (Subtopic 5), push arrow and strategy text (Subtopic 6).
%
%   ov : struct with fields text, zmp [x y], cp [x y], push (true/false), resetTrail,
%        layer = what to draw: 1 robot only, 2 + CoM, 3 + support polygon (and more),
%        pushDir (optional) = [dx dy] direction of the push arrow, default [1 0]

H = ax.UserData;
if ~isstruct(H)                           % first call: create everything once
    show(robot, q, 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true, 'Frames', 'off');
    hold(ax, 'on');
    H.poly  = patch(ax, 'XData', nan, 'YData', nan, 'ZData', nan, 'FaceColor', 'g', ...
                    'FaceAlpha', 0.35, 'EdgeColor', 'k');
    H.trail = plot3(ax, nan, nan, nan, 'r-', 'LineWidth', 1.5);
    H.drop  = plot3(ax, nan, nan, nan, 'k--');
    H.proj  = plot3(ax, nan, nan, nan, 'kx', 'MarkerSize', 12, 'LineWidth', 2);
    H.com   = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
    H.zmp   = plot3(ax, nan, nan, nan, 's', 'MarkerSize', 9, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k');
    H.cp    = plot3(ax, nan, nan, nan, 'd', 'MarkerSize', 10, 'MarkerFaceColor', [0 0.7 0], 'MarkerEdgeColor', 'k');
    H.push  = quiver3(ax, nan, nan, nan, nan, nan, nan, 0, 'Color', [0.85 0 0], 'LineWidth', 4, 'MaxHeadSize', 2);
    % Legend on static stand-in markers, so moving the real ones does not redraw it
    p1 = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
    p2 = plot3(ax, nan, nan, nan, 's', 'MarkerSize', 9, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k');
    p3 = plot3(ax, nan, nan, nan, 'd', 'MarkerSize', 10, 'MarkerFaceColor', [0 0.7 0], 'MarkerEdgeColor', 'k');
    p4 = patch(ax, 'XData', nan, 'YData', nan, 'ZData', nan, 'FaceColor', [0.2 0.8 0.2], 'FaceAlpha', 0.35, 'EdgeColor', 'k');
    legend(ax, [p1 p2 p3 p4], {'CoM', 'ZMP', 'Capture point', 'Support polygon'}, ...
           'Location', 'northeast', 'AutoUpdate', 'off');
    axis(ax, [-0.15 0.25 -0.2 0.2 0 0.45]);
    view(ax, 25, 12);                     % side view: forward lean and step are clear
    H.info = text(ax, 0.01, 0.99, '', 'Units', 'normalized', 'VerticalAlignment', 'top', ...
                  'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', 'w', 'Margin', 3);
    ax.UserData = H;
end

show(robot, q, 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true, 'Frames', 'off');

[com, comFloor] = computeCoM(robot, info, q);            % Subtopic 2
poly   = supportPolygon(robot, info, q);                 % Subtopic 3
margin = stabilityMargin(comFloor, poly);
zf = info.floorZ + 0.001;

col = [0.2 0.8 0.2];                                     % green = stable
if margin < 0
    col = [0.9 0.2 0.2];                                 % red = CoM outside
end
set(H.poly, 'XData', poly(:, 1), 'YData', poly(:, 2), ...
    'ZData', zf * ones(size(poly, 1), 1), 'FaceColor', col);

if ov.resetTrail
    set(H.trail, 'XData', com(1), 'YData', com(2), 'ZData', com(3));
else
    set(H.trail, 'XData', [H.trail.XData com(1)], 'YData', [H.trail.YData com(2)], ...
        'ZData', [H.trail.ZData com(3)]);
end

set(H.com,  'XData', com(1), 'YData', com(2), 'ZData', com(3));
set(H.proj, 'XData', com(1), 'YData', com(2), 'ZData', zf);
set(H.drop, 'XData', [com(1) com(1)], 'YData', [com(2) com(2)], 'ZData', [com(3) zf]);
set(H.zmp,  'XData', ov.zmp(1), 'YData', ov.zmp(2), 'ZData', zf);
set(H.cp,   'XData', ov.cp(1),  'YData', ov.cp(2),  'ZData', zf);

if ov.push        % red arrow hitting the robot (default: from behind, pushing +x)
    d = [1 0];
    if isfield(ov, 'pushDir'), d = ov.pushDir; end
    set(H.push, 'XData', com(1) - 0.14*d(1), 'YData', com(2) - 0.14*d(2), 'ZData', com(3) + 0.06, ...
        'UData', 0.09*d(1), 'VData', 0.09*d(2), 'WData', 0);
else
    set(H.push, 'XData', nan, 'YData', nan, 'ZData', nan);
end

% Show only the layers this part has introduced so far (only when the part changes,
% because changing visibility makes the legend redraw)
if ~isfield(H, 'layer') || H.layer ~= ov.layer
    onoff = ["off" "on"];
    set([H.com H.proj H.drop H.trail], 'Visible', onoff((ov.layer >= 2) + 1));
    set(H.poly, 'Visible', onoff((ov.layer >= 3) + 1));
    H.layer = ov.layer;
    ax.UserData = H;
end

% Text box in the top-left corner of the robot plot
if ov.layer >= 3
    H.info.String = {ov.text, sprintf('CoM stability margin: %+.1f mm', 1000 * margin)};
elseif ov.layer == 2
    H.info.String = {ov.text, sprintf('CoM = [%.3f  %.3f  %.3f] m', com)};
else
    H.info.String = ov.text;
end
drawnow limitrate
end
