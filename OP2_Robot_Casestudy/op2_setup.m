function [robot, info] = op2_setup()
%OP2_SETUP  Subtopic 1: ROBOTIS OP2 model, frames and conventions (shared by every part).
%   World frame = the floor: x forward (the way the robot faces), y to its left, z up.
%   The stock OP2 base (base_link) is fixed under the torso, so moving a leg moves the
%   foot, not the body. We hang OP2 under a massless floating base so the torso can
%   move while the feet stay on the floor:
%       world -> base_x -> base_y -> base_z (sliders) -> base_pitch (torso lean) -> OP2
%   Configuration: q = [x; y; z; pitch; 24 OP2 joints]
%
%   robot : rigidBodyTree with the floating base
%   info  : names, joint indices, foot geometry, home pose and the leg IK solver

op2 = loadrobot("robotisOP2", "DataFormat", "column");

%% Floating base
robot = rigidBodyTree("DataFormat", "column");
names = ["base_x", "base_y", "base_z", "base_pitch"];
types = ["prismatic", "prismatic", "prismatic", "revolute"];
axesB = [1 0 0; 0 1 0; 0 0 1; 0 1 0];
parent = robot.BaseName;
for i = 1:4
    body = rigidBody(names(i));
    body.Mass = 0;                        % massless, so the CoM is unchanged
    body.Inertia = zeros(1, 6);
    jnt = rigidBodyJoint(names(i) + "_jnt", types(i));
    jnt.JointAxis = axesB(i, :);
    jnt.PositionLimits = [-1 1];
    body.Joint = jnt;
    addBody(robot, body, parent);
    parent = names(i);
end
addSubtree(robot, "base_pitch", op2);

%% Joint names in configuration order
info.jointNames = strings(0, 1);
bodies = robot.Bodies;                    % read once (each read copies every body)
for i = 1:numel(bodies)
    j = bodies{i}.Joint;
    if ~strcmp(j.Type, 'fixed')
        info.jointNames(end+1, 1) = string(j.Name);
    end
end
idx = @(n) arrayfun(@(s) find(info.jointNames == s), n);

% OP2 leg joints (all rotate about their local z axis):
%   j_pelvis = hip yaw, j_thigh1 = hip roll, j_thigh2 = hip pitch,
%   j_tibia = knee, j_ankle1 = ankle pitch, j_ankle2 = ankle roll
% The right leg's pitch joints are mirrored: a forward knee bend is
%   left (+a, -2a, -a) and right (-a, +2a, +a) for (thigh2, tibia, ankle1).
legJoints = ["j_pelvis", "j_thigh1", "j_thigh2", "j_tibia", "j_ankle1", "j_ankle2"];
info.nb    = 4;                           % number of floating-base joints
info.base  = idx(names(1:3) + "_jnt");    % x, y, z sliders
info.pitch = idx("base_pitch_jnt");       % torso lean (+ = forward)
info.legL  = idx(legJoints + "_l");
info.legR  = idx(legJoints + "_r");

%% Bodies and foot geometry
info.torso = "MP_BODY";
info.footL = "MP_ANKLE2_L";               % left foot  (y > 0)
info.footR = "MP_ANKLE2_R";               % right foot (y < 0)

q0 = homeConfiguration(robot);
info.TfootL0 = getTransform(robot, q0, info.footL);
info.TfootR0 = getTransform(robot, q0, info.footR);
info.ankleHeight = info.TfootL0(3, 4);    % ankle height when the sole is on the floor

% Sole footprint, measured from the OP2 meshes in the home pose
info.floorZ      = 0.0083;                % soles sit 8 mm above base_link
info.footSize    = [0.105 0.066];         % sole length x width (m)
info.soleOffsetL = [0  0.0105];           % sole centre relative to the ankle (x, y)
info.soleOffsetR = [0 -0.0105];

%% Leg IK solver (Subtopic 4)
% Runs on the stock fixed-base OP2, so it only sees the six joints of a leg.
info.op2 = op2;
info.ik  = inverseKinematics('RigidBodyTree', op2);
info.ik.SolverParameters.AllowRandomRestart = false;
info.ik.SolverParameters.MaxIterations = 300;   % fail fast if a target is out of reach
info.ikWeights = ones(1, 6);

%% Home pose: knees slightly bent forward (straight knees are singular for IK)
qSeed = q0;
qSeed(info.legL(3:5)) = [ 0.2 -0.4 -0.2];
qSeed(info.legR(3:5)) = [-0.2  0.4  0.2];
comStraight = centerOfMass(robot, q0);
info.qHome = solvePosture(robot, info, comStraight - [0; 0; 0.01], ...
                          info.TfootL0, info.TfootR0, qSeed);
end
