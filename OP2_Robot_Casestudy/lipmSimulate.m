function sim = lipmSimulate(x0, v0, omega, t, copFun)
%LIPMSIMULATE  Subtopic 5: linear inverted pendulum after a push (forward direction).
%   CoM dynamics:  x'' = omega^2 * (x - p)
%   p is the ZMP (centre of pressure) under the feet, chosen by copFun(t, x, v).
%   A push is modelled as a sudden CoM velocity v0.
%
%   sim.t, sim.x (CoM), sim.v (velocity), sim.zmp, sim.xi (capture point)

n = numel(t);
x = zeros(1, n);  v = zeros(1, n);  p = zeros(1, n);
x(1) = x0;  v(1) = v0;
for k = 1:n
    p(k) = copFun(t(k), x(k), v(k));
    if k < n
        dt = t(k+1) - t(k);
        v(k+1) = v(k) + omega^2 * (x(k) - p(k)) * dt;    % semi-implicit Euler
        x(k+1) = x(k) + v(k+1) * dt;
    end
end

sim.t = t;  sim.x = x;  sim.v = v;  sim.zmp = p;
sim.xi = capturePoint(x, v, omega);
end
