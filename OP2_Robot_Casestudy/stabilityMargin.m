function m = stabilityMargin(p, poly)
%STABILITYMARGIN  Subtopic 3: signed distance from a point to the support polygon edge.
%   m > 0 : inside, statically stable (distance to the nearest edge)
%   m = 0 : on the edge
%   m < 0 : outside, unstable (distance to the polygon)

p = p(1:2);  p = p(:)';
d = inf;
for i = 1:size(poly, 1) - 1
    a = poly(i, :);
    b = poly(i + 1, :);
    t = max(0, min(1, dot(p - a, b - a) / dot(b - a, b - a)));   % closest point on the edge
    d = min(d, norm(p - (a + t * (b - a))));
end
if inpolygon(p(1), p(2), poly(:, 1), poly(:, 2))
    m = d;
else
    m = -d;
end
end
