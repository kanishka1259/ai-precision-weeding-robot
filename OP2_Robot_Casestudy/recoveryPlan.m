function plan = recoveryPlan(strategy, x0, v0, P)
%RECOVERYPLAN  Subtopic 6: CoM, ZMP, torso and foot motion for one recovery.
%   Uses the inverted pendulum (Subtopic 5) with a capture-point controller:
%       p = xiRef + (1 + k/omega) * (xi - xiRef),  clipped to the support polygon
%   which drives the capture point xi back to xiRef.
%
%   ankle : CoP limited to the feet
%   hip   : torso leans forward, which lets the CoP reach hipExtra further for a while
%   step  : right foot steps forward so the capture point lands inside the new polygon
%
%   P holds omega, dt, tEnd, k, footBack, footFront, xRest, hipExtra, hipHold,
%   maxPitch, tLand, landBack, maxStep, lift, yShift.

t = 0:P.dt:P.tEnd;
n = numel(t);
plan.strategy = strategy;
plan.pitch  = zeros(1, n);                % torso lean (rad)
plan.yShift = zeros(1, n);                % sideways CoM shift (m)
plan.swingX = zeros(1, n);                % right-foot offset from its start (m)
plan.swingZ = zeros(1, n);
plan.front  = P.footFront * ones(1, n);   % how far forward the CoP may go
plan.xLand  = 0;

switch strategy
    case "ankle"
        cop = @(tt, x, v) ctrl(x + v/P.omega, P.xRest, P, P.footBack, P.footFront);
        sim = lipmSimulate(x0, v0, P.omega, t, cop);

    case "hip"
        shape = @(tt) min(tt/0.04, 1) .* (tt <= P.hipHold) + ...
                      max(1 - (tt - P.hipHold)/0.3, 0) .* (tt > P.hipHold);
        cop = @(tt, x, v) ctrl(x + v/P.omega, P.xRest, P, P.footBack, ...
                               P.footFront + P.hipExtra * shape(tt));
        sim = lipmSimulate(x0, v0, P.omega, t, cop);
        plan.pitch = P.maxPitch * shape(t);
        plan.front = P.footFront + P.hipExtra * shape(t);

    case "step"
        % Phase 1: right foot in the air, CoP limited to the left foot
        in1 = t <= P.tLand;
        cop1 = @(tt, x, v) ctrl(x + v/P.omega, P.xRest, P, P.footBack, P.footFront);
        s1 = lipmSimulate(x0, v0, P.omega, t(in1), cop1);

        % Land so the capture point is inside the new foot
        xLand = min(s1.xi(end) - P.landBack, P.maxStep);
        front2 = P.footFront + xLand;
        xRest2 = P.xRest + xLand / 2;               % settle between the feet

        % Phase 2: both feet down, larger polygon
        t2 = [t(find(in1, 1, 'last')) t(~in1)];
        cop2 = @(tt, x, v) ctrl(x + v/P.omega, xRest2, P, P.footBack, front2);
        s2 = lipmSimulate(s1.x(end), s1.v(end), P.omega, t2, cop2);

        sim.x   = [s1.x   s2.x(2:end)];
        sim.v   = [s1.v   s2.v(2:end)];
        sim.zmp = [s1.zmp s2.zmp(2:end)];
        sim.xi  = [s1.xi  s2.xi(2:end)];

        % Swing foot: forward and up, then down on the landing spot
        s = min(t / P.tLand, 1);
        s = (1 - cos(pi * s)) / 2;
        plan.swingX = xLand * s;
        plan.swingZ = P.lift * sin(pi * s) .* (t <= P.tLand);
        plan.front(~in1) = front2;
        plan.xLand = xLand;

        % CoM leans over the left foot while the right foot is in the air
        plan.yShift = P.yShift * (sin(pi * min(t / (P.tLand + 0.3), 1)));
end

plan.t   = t;
plan.x   = sim.x;
plan.v   = sim.v;
plan.zmp = sim.zmp;
plan.xi  = sim.xi;
end

function p = ctrl(xi, xiRef, P, pMin, pMax)
% Capture-point controller, CoP clipped to what the feet (and hips) allow
p = xiRef + (1 + P.k / P.omega) * (xi - xiRef);
p = min(max(p, pMin), pMax);
end
