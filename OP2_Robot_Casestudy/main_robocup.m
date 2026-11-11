2%% Real-world use case: ROBOTIS OP2 keeping its balance in a RoboCup soccer match
% The ROBOTIS OP2 (DARwIn-OP2) plays in the RoboCup Humanoid KidSize league.
% In a match it must walk, kick and survive bumps from opponents without falling.
% This script plays one short match situation, built from the six subtopics:
%
%   Event 0  Kick-off: crouch into the ready stance          (Subtopic 4)
%   Event 1  Walk to the ball: 4 statically stable steps      (Subtopics 2, 3, 4)   NEW
%   Event 2  Kick the ball on one leg -> goal                 (Subtopics 2, 3, 4)   NEW
%   Event 3  Side bump from an opponent: ankle strategy       (Subtopics 5, 6, 4)   NEW
%   Event 4  Hard charge from behind: capture point -> step   (Subtopics 5, 6, 4)
%
% Run this file. All .m files in this folder must stay together.
% One window: robot on the pitch (left), live top view and live stability margin (right).

clc; clear; close all;
holdTime = 0.5;                 % pause between events in seconds (0 = no pauses)
drawEvery = 2;                  % draw every 2nd frame (1 = smoothest, 3 = fastest)

%% Setup: robot, window and pitch
[robot, info] = op2_setup();               % Subtopic 1
TL = info.TfootL0;                         % left foot pose on the floor
TR = info.TfootR0;                         % right foot pose on the floor
q  = info.qHome;

walkAdvance = [0.04 0.08 0.08 0.04];       % how far each step moves its foot (R, L, R, L)
ball.r = 0.033;                            % ball radius (m)
ballX = TR(1,4) + sum(walkAdvance)/2 + 0.053 + ball.r + 0.01;   % in front of the right toe after walking
ballY = TR(2,4);
goalX = ballX + 0.36;                      % goal line

fig = figure("Name", "RoboCup use case: OP2 balance during a match", "Color", "w", ...
             "Position", [40 60 1400 760]);
tl  = tiledlayout(fig, 2, 3, "TileSpacing", "compact", "Padding", "compact");
tl.Title.FontSize = 15;  tl.Title.FontWeight = "bold";
ax    = nexttile(tl, 1, [2 2]);            % robot on the pitch
axTop = nexttile(tl, 3);                   % top view
axBot = nexttile(tl, 6);                   % stability margin over the match

S = struct('robot', robot, 'info', info, 'ax', ax, 'tl', tl, 'axTop', axTop, 'axBot', axBot, ...
           't', 0, 'dt', 0.03, 'minMargin', inf, 'drawEvery', drawEvery, 'count', 0, 'kickArmed', false, 'kickSpeed', 0.7, 'goalX', goalX);

% Robot drawing first, then the pitch, goal and ball around it
animateFrame(ax, robot, info, q, struct('text', "Kick-off", 'layer', 3, 'zmp', [nan nan], ...
             'cp', [nan nan], 'push', false, 'resetTrail', true));
axis(ax, [-0.15 goalX + 0.06 -0.25 0.25 0 0.45]);
view(ax, 30, 14);
S = addPitch(S, ballX, ballY);

% Top view (live): support polygon, CoM path, footprints, ball, goal
hold(axTop, 'on'); grid(axTop, 'on'); axis(axTop, 'equal');
xlim(axTop, [-0.15 goalX + 0.06]);  ylim(axTop, [-0.15 0.15]);
title(axTop, "Top view (live): footprints, CoM path, ball");
xlabel(axTop, "x (m)");  ylabel(axTop, "y (m)");
hGoal = plot(axTop, [goalX goalX], [-0.12 0.12], 'k-', 'LineWidth', 4);
S.hTopPoly = patch(axTop, nan, nan, [0.2 0.8 0.2], 'FaceAlpha', 0.4);
S.hTopPath = plot(axTop, nan, nan, 'r-', 'LineWidth', 1.5);
S.hTopCom  = plot(axTop, nan, nan, 'ro', 'MarkerFaceColor', 'r');
S.hTopBall = plot(axTop, ballX, ballY, 'o', 'MarkerSize', 9, 'MarkerFaceColor', [1 0.55 0], 'MarkerEdgeColor', 'k');
% legend on static stand-ins (so live updates do not redraw it)
lp = [patch(axTop, nan, nan, [0.2 0.8 0.2], 'FaceAlpha', 0.4), plot(axTop, nan, nan, 'r-', 'LineWidth', 1.5), ...
      plot(axTop, nan, nan, 'ro', 'MarkerFaceColor', 'r'), ...
      plot(axTop, nan, nan, 'o', 'MarkerSize', 9, 'MarkerFaceColor', [1 0.55 0], 'MarkerEdgeColor', 'k')];
legend(axTop, [hGoal lp], ["goal", "support polygon", "CoM path", "CoM", "ball"], ...
       'Location', 'southoutside', 'NumColumns', 5, 'AutoUpdate', 'off');
drawFootprint(axTop, info, TL, 1);
drawFootprint(axTop, info, TR, 2);

% Stability margin over the whole match (live)
hold(axBot, 'on'); grid(axBot, 'on');
title(axBot, "CoM stability margin during the match (live)");
xlabel(axBot, "match time (s)");  ylabel(axBot, "margin (mm)");
yline(axBot, 0, 'r--', 'HandleVisibility', 'off');
S.hMargin = plot(axBot, nan, nan, 'b', 'LineWidth', 1.5);

evName = ["Kick-off", "Walk to the ball", "Kick", "Side bump", "Charge from behind"];
evMin  = nan(1, numel(evName));            % lowest margin in each event

%% Event 0 - Kick-off: crouch into the ready stance (Subtopic 4)
S = startEvent(S, 0, "Kick-off: crouch into the ready stance", "Subtopic 4");
com0 = centerOfMass(robot, q);
zc = com0(3) - 0.02;                       % CoM height used during the match
comC = [(TL(1,4) + TR(1,4))/2; (TL(2,4) + TR(2,4))/2; zc];
[S, q] = move(S, q, comC, TL, TR, 12, "Ready stance: CoM 2 cm lower, centred between the feet");
evMin(1) = S.minMargin;  pause(holdTime);

%% Event 1 - Walk to the ball: static walking (Subtopics 2, 3, 4)   NEW ACTION
% Each step: move the CoM over the stance foot (weight shift), lift the other
% foot, swing it forward, put it down. The CoM stays inside the support polygon
% the whole time, so the robot could stop at any moment without falling.
S = startEvent(S, 1, "Walk to the ball (static walking)", "Subtopics 2, 3, 4");
feet = {TL, TR};                           % 1 = left, 2 = right
side = 'RLRL';
for i = 1:numel(side)
    sw = 1 + (side(i) == 'R');             % swing foot index
    st = 3 - sw;                           % stance foot index
    names = ["left", "right"];
    comS = [feet{st}(1,4); feet{st}(2,4); zc];          % CoM over the stance ankle
    [S, q] = move(S, q, comS, feet{1}, feet{2}, 8, ...
        sprintf("Step %d of 4: weight onto the %s foot", i, names(st)));

    mid  = feet{sw};  mid(1,4)  = mid(1,4)  + walkAdvance(i)/2;  mid(3,4) = mid(3,4) + 0.02;
    land = feet{sw};  land(1,4) = land(1,4) + walkAdvance(i);
    txt  = sprintf("Step %d of 4: swing the %s foot forward %.0f cm", i, names(sw), 100*walkAdvance(i));
    f2 = feet;  f2{sw} = mid;
    [S, q] = move(S, q, comS, f2{1}, f2{2}, 6, txt);
    feet{sw} = land;
    [S, q] = move(S, q, comS, feet{1}, feet{2}, 6, txt);
    drawFootprint(axTop, info, land, sw);
end
TL = feet{1};  TR = feet{2};
comC = [(TL(1,4) + TR(1,4))/2; (TL(2,4) + TR(2,4))/2; zc];
[S, q] = move(S, q, comC, TL, TR, 10, "Arrived: CoM back between the feet");
fprintf("Walked %.0f cm. Lowest margin while walking: %.1f mm\n", 100*sum(walkAdvance)/2, 1000*S.minMargin);
evMin(2) = S.minMargin;  pause(holdTime);

%% Event 2 - Kick the ball on one leg (Subtopics 2, 3, 4)   NEW ACTION
% Balance on the left leg (CoM over the left ankle), wind up the right foot and
% swing it through the ball. The support polygon is only the left sole here.
S = startEvent(S, 2, "Kick the ball on one leg", "Subtopics 2, 3, 4");
comK = [TL(1,4); TL(2,4); zc];
off  = @(T, dx, dz) T + [zeros(3) [dx; 0; dz]; 0 0 0 0];
[S, q] = move(S, q, comK, TL, TR, 8, "Weight onto the left (support) leg");
S.kickArmed = true;
[S, q] = move(S, q, comK, TL, off(TR, -0.03, 0.025), 8, "Lift the right foot and wind up");
[S, q] = move(S, q, comK, TL, off(TR,  0.05, 0.02),   5, "KICK!");
[S, q] = move(S, q, comK, TL, off(TR,  0,    0.02),   6, "Follow-through, bring the foot back");
[S, q] = move(S, q, comK, TL, TR, 5, "Foot down");
[S, q] = move(S, q, comC, TL, TR, 8, "Both feet: CoM back to the centre");
while S.ball.v > 0                         % watch the ball finish rolling
    S = frame(S, q, "Watching the ball");
end
evMin(3) = S.minMargin;  pause(holdTime);

%% Event 3 - Side bump from an opponent (Subtopics 5, 6, 4)   NEW ACTION
% Same inverted pendulum as Subtopic 5, solved sideways (y). The capture point
% stays inside the feet, so the ankle strategy (moving the ZMP) is enough.
S = startEvent(S, 3, "Side bump from an opponent", "Subtopics 5, 6, 4");
com  = centerOfMass(robot, q);
poly = supportPolygon(robot, info, q);
omega = sqrt(9.81 / (com(3) - info.floorZ));

P = struct('omega', omega, 'dt', 0.005, 'tEnd', 1.5, 'k', 4, 'hipExtra', 0.03, 'hipHold', 0.4, ...
           'maxPitch', 0.35, 'tLand', 0.12, 'landBack', 0.03, 'maxStep', 0.10, 'lift', 0.02, 'yShift', 0.02);
Py = P;                                     % pendulum sideways: "x" in the plan is y here
Py.footBack = min(poly(:, 2));  Py.footFront = max(poly(:, 2));  Py.xRest = com(2);
vy  = -0.25;                                % opponent pushes towards the robot's right (-y)
xiY = capturePoint(com(2), vy, omega);
fprintf("Side push %.2f m/s -> capture point y = %.3f m (feet span y %.3f to %.3f m): ankle strategy\n", ...
        abs(vy), xiY, Py.footBack, Py.footFront);
plan = recoveryPlan("ankle", com(2), vy, Py);
for k = 1:8:numel(plan.t)                  % one frame every 0.04 s
    comT = [com(1); plan.x(k); com(3)];
    q = solvePosture(robot, info, comT, TL, TR, q);
    S = frame(S, q, sprintf("Side bump %.2f m/s: ankle strategy sideways  (t = %.2f s)", abs(vy), plan.t(k)), ...
              struct('zmp', [com(1) plan.zmp(k)], 'cp', [com(1) plan.xi(k)], ...
                     'push', plan.t(k) < 0.08, 'pushDir', [0 -1]));
end
evMin(4) = S.minMargin;  pause(holdTime);

%% Event 4 - Hard charge from behind (Subtopics 5, 6, 4)
% The capture point lands far in front of the feet, so the robot must step.
S = startEvent(S, 4, "Hard charge from behind", "Subtopics 5, 6, 4");
com  = centerOfMass(robot, q);
poly = supportPolygon(robot, info, q);
P.footBack = min(poly(:, 1));  P.footFront = max(poly(:, 1));  P.xRest = com(1);
v0  = 0.60;
xi0 = capturePoint(P.xRest, v0, omega);
strategy = chooseStrategy(xi0, P.footFront, P.hipExtra);
fprintf("Push %.2f m/s -> capture point x = %.3f m (feet end at %.3f m): %s strategy\n", ...
        v0, xi0, P.footFront, strategy);
plan = recoveryPlan(strategy, P.xRest, v0, P);
for k = 1:8:numel(plan.t)                  % one frame every 0.04 s
    comT = [plan.x(k); com(2) + plan.yShift(k); com(3)];
    TRk = TR;  TRk(1,4) = TR(1,4) + plan.swingX(k);  TRk(3,4) = TR(3,4) + plan.swingZ(k);
    q = solvePosture(robot, info, comT, TL, TRk, q, plan.pitch(k));
    S = frame(S, q, sprintf("Charge %.2f m/s: %s strategy  (t = %.2f s)", v0, upper(strategy), plan.t(k)), ...
              struct('zmp', [plan.zmp(k) comT(2)], 'cp', [plan.xi(k) comT(2)], 'push', plan.t(k) < 0.08));
end
if strategy == "step"                       % close the stance: bring the left foot forward too
    TR = off(TR, plan.xLand, 0);
    drawFootprint(axTop, info, TR, 2);
    comF = [TR(1,4) - 0.03; TR(2,4); zc];   % over the back of the right sole
    [S, q] = move(S, q, comF, TL, TR, 8, "Weight onto the front (right) foot");
    [S, q] = move(S, q, comF, off(TL, plan.xLand/2, 0.02), TR, 6, "Bring the left foot forward");
    TL = off(TL, plan.xLand, 0);
    [S, q] = move(S, q, comF, TL, TR, 6, "Bring the left foot forward");
    drawFootprint(axTop, info, TL, 1);
    comC = [(TL(1,4) + TR(1,4))/2; (TL(2,4) + TR(2,4))/2; zc];
    [S, q] = move(S, q, comC, TL, TR, 10, "Balanced again, ready to play");
end
evMin(5) = S.minMargin;

%% Full time: match report
S.tl.Title.String = "Full time: the robot walked, scored and stayed on its feet after every bump";
ax.UserData.info.String = "Full time";
fprintf("\n===== Match report =====\n");
fprintf("%-22s %-18s\n", "Event", "Lowest margin (mm)");
for i = 1:numel(evName)
    fprintf("%-22s %+8.1f\n", evName(i), 1000 * evMin(i));
end
fprintf("Goal scored: %s.  Fell over: no.\n", string(S.ball.scored));

%% Local helpers
function S = startEvent(S, n, name, subtopics)
% New event: big title, marker on the margin graph, heading in the Command Window
S.tl.Title.String = sprintf("RoboCup match - Event %d: %s  (%s)", n, name, subtopics);
xline(S.axBot, S.t, ':', sprintf("E%d", n), 'HandleVisibility', 'off');
fprintf("\n===== Event %d: %s =====\n", n, name);
S.minMargin = inf;
end

function [S, q] = move(S, q, comGoal, TLg, TRg, n, txt)
% Smooth IK move (Subtopic 4), one frame per step
Q = planMove(S.robot, S.info, q, comGoal, TLg, TRg, n);
for k = 1:n
    S = frame(S, Q(:, k), txt);
end
q = Q(:, end);
end

function S = frame(S, q, txt, extra)
% Draw one frame and advance the match clock: robot + overlays, ball, top view, margin
ov = struct('text', txt, 'layer', 3, 'zmp', [nan nan], 'cp', [nan nan], 'push', false, 'resetTrail', false);
if nargin > 3
    for f = string(fieldnames(extra))'
        ov.(f) = extra.(f);
    end
end
S.count = S.count + 1;
if mod(S.count, S.drawEvery) == 0 || ov.push       % draw this frame
    [m, com, poly] = animateFrame(S.ax, S.robot, S.info, q, ov);
else                                               % skip drawing, still check stability
    [com, comFloor] = computeCoM(S.robot, S.info, q);
    poly = supportPolygon(S.robot, S.info, q);
    m = stabilityMargin(comFloor, poly);
end

% Kick: the right toe reaches the ball while the foot is in the air
if S.kickArmed
    Tr = getTransform(S.robot, q, S.info.footR);
    toe = Tr(1,4) + 0.053;
    if toe >= S.ball.x - S.ball.r && Tr(3,4) > S.info.ankleHeight + 0.005
        S.ball.v = S.kickSpeed;
        S.kickArmed = false;
        fprintf("Ball kicked at %.2f m/s\n", S.ball.v);
    end
end

% Ball rolls with friction and stops in the net
if S.ball.v > 0
    S.ball.x = S.ball.x + S.ball.v * S.dt;
    S.ball.v = max(S.ball.v - S.ball.decel * S.dt, 0);
    if S.ball.x > S.goalX + 0.04
        S.ball.v = 0;
    end
    set(S.ball.h, 'XData', S.ball.X0 + S.ball.x);
    set(S.hTopBall, 'XData', S.ball.x);
    if ~S.ball.scored && S.ball.x - S.ball.r > S.goalX
        S.ball.scored = true;
        text(S.ax, S.goalX, 0, 0.25, 'GOAL!', 'FontSize', 24, 'FontWeight', 'bold', ...
             'Color', [0.85 0 0], 'HorizontalAlignment', 'center');
        fprintf("GOAL!\n");
    end
end

% Top view and margin graph
col = [0.2 0.8 0.2];
if m < 0, col = [0.9 0.2 0.2]; end
set(S.hTopPoly, 'XData', poly(:,1), 'YData', poly(:,2), 'FaceColor', col);
set(S.hTopPath, 'XData', [S.hTopPath.XData com(1)], 'YData', [S.hTopPath.YData com(2)]);
set(S.hTopCom,  'XData', com(1), 'YData', com(2));
S.t = S.t + S.dt;
set(S.hMargin, 'XData', [S.hMargin.XData S.t], 'YData', [S.hMargin.YData 1000*m]);
S.minMargin = min(S.minMargin, m);
end

function S = addPitch(S, ballX, ballY)
% Grass, goal and ball in the robot plot
ax = S.ax;  z = S.info.floorZ;  gx = S.goalX;
patch(ax, [-0.15 gx+0.06 gx+0.06 -0.15], [-0.25 -0.25 0.25 0.25], (z - 0.002)*[1 1 1 1], ...
      [0.55 0.8 0.45], 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot3(ax, [gx gx], [-0.25 0.25], [z z], 'w-', 'LineWidth', 3, 'HandleVisibility', 'off');
gy = 0.12;  gh = 0.15;
plot3(ax, [gx gx gx gx], [-gy -gy gy gy], [z z+gh z+gh z], 'Color', [0.97 0.97 0.97], ...
      'LineWidth', 5, 'HandleVisibility', 'off');
r = 0.033;
[X, Y, Z] = sphere(20);
S.ball = struct('x', ballX, 'y', ballY, 'v', 0, 'r', r, 'decel', 0.5, 'X0', r*X, 'scored', false);
S.ball.h = surf(ax, r*X + ballX, r*Y + ballY, r*Z + z + r, 'FaceColor', [1 0.55 0], ...
                'EdgeColor', 'none', 'HandleVisibility', 'off');
end

function drawFootprint(axTop, info, T, foot)
% Grey sole outline on the top view where a foot has landed
offs = [info.soleOffsetL; info.soleOffsetR];
c = T(1:2, 4)' + offs(foot, :);
L = info.footSize(1)/2;  W = info.footSize(2)/2;
patch(axTop, c(1) + [L -L -L L], c(2) + [W W -W -W], [0.6 0.6 0.6], 'FaceAlpha', 0.3, ...
      'EdgeColor', [0.4 0.4 0.4], 'HandleVisibility', 'off');
end
