function poly = supportPolygon(robot, info, q)
%SUPPORTPOLYGON  Subtopic 3: convex hull of every sole touching the floor.
%   poly : closed polygon (N x 2, first point repeated at the end) in floor x, y
%
%   The corners are placed with the floor-aligned sole footprint. The ankle frame
%   itself is rotated (its x axis points up), so corners must not be expressed in it.

L = info.footSize(1) / 2;
W = info.footSize(2) / 2;
corners = [L W; -L W; -L -W; L -W];
feet = {info.footL, info.soleOffsetL; info.footR, info.soleOffsetR};

pts = [];
for i = 1:2
    T = getTransform(robot, q, feet{i, 1});
    if T(3, 4) < info.ankleHeight + 0.002          % this foot is on the floor
        centre = T(1:2, 4)' + feet{i, 2};
        pts = [pts; corners + centre]; %#ok<AGROW>
    end
end
k = convhull(pts(:, 1), pts(:, 2));
poly = pts(k, :);
end
