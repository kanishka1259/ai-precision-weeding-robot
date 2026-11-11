function [q, com, ok] = solvePosture(robot, info, comTarget, TfootL, TfootR, qInit, pitch)
%SOLVEPOSTURE  Subtopic 4: joint angles that put the CoM at comTarget with the feet fixed.
%   [q, com, ok] = solvePosture(robot, info, comTarget, TfootL, TfootR, qInit, pitch)
%
%   MATLAB's IK has no CoM constraint, so the torso stands in for the CoM:
%     1. place the torso (floating base) with the requested lean 'pitch'
%     2. solve IK for each leg so its foot lands on its target pose
%     3. measure the CoM and shift the torso by the CoM error, repeat
%
%   comTarget      : 3x1 desired CoM in the world (floor) frame
%   TfootL, TfootR : 4x4 desired foot poses (lift a foot by raising its z)
%   qInit          : starting guess (use the previous posture)
%   pitch          : torso lean in rad, + = forward (optional, default 0)

if nargin < 7
    pitch = 0;
end
comTarget = comTarget(:);
q = qInit(:);
q(info.pitch) = pitch;
base = q(info.base);
com  = centerOfMass(robot, q);
base = base + (comTarget - com);          % first guess: move the torso as much as the CoM must move

for iter = 1:20
    % Foot targets as seen from OP2's base_link
    Tbase = trvec2tform(base') * axang2tform([0 1 0 pitch]);
    qLegs = q(info.nb+1:end);
    [qLegs, infoL] = info.ik(char(info.footL), Tbase \ TfootL, info.ikWeights, qLegs);
    [qLegs, infoR] = info.ik(char(info.footR), Tbase \ TfootR, info.ikWeights, qLegs);
    q = [base; pitch; qLegs];

    com = centerOfMass(robot, q);
    err = comTarget - com;
    if norm(err) < 5e-4                   % 0.5 mm is plenty
        break
    end
    base = base + err;                    % correct the torso by the CoM error
end

ok = norm(err) < 1e-3 && infoL.PoseErrorNorm < 1e-3 && infoR.PoseErrorNorm < 1e-3;
end
