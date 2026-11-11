function [Q, ok] = planMove(robot, info, q, comGoal, TfootLGoal, TfootRGoal, nSteps)
%PLANMOVE  Subtopic 4: move the CoM and feet smoothly from where they are to a goal.
%   Interpolates the CoM and foot positions and calls solvePosture at every step.
%   Returns the joint trajectory Q (one column per frame).

com0 = centerOfMass(robot, q);
TL0  = getTransform(robot, q, info.footL);
TR0  = getTransform(robot, q, info.footR);

Q  = zeros(numel(q), nSteps);
ok = true;
for k = 1:nSteps
    s = (1 - cos(pi * k / nSteps)) / 2;   % 0 -> 1 with smooth start and stop

    comK = com0 + s * (comGoal(:) - com0);
    TL = TL0;  TL(1:3, 4) = TL0(1:3, 4) + s * (TfootLGoal(1:3, 4) - TL0(1:3, 4));
    TR = TR0;  TR(1:3, 4) = TR0(1:3, 4) + s * (TfootRGoal(1:3, 4) - TR0(1:3, 4));

    [q, ~, okK] = solvePosture(robot, info, comK, TL, TR, q);
    ok = ok && okK;
    Q(:, k) = q;
end

if ~ok
    warning('planMove: some frames did not reach the CoM/foot targets.');
end
end
