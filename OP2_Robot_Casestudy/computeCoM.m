function [com, comFloor] = computeCoM(robot, info, q)
%COMPUTECOM  Subtopic 2: centre of mass and its projection on the floor.
%   com      : 3x1 CoM in the world frame (m)
%   comFloor : 3x1 point on the floor straight below the CoM

com = centerOfMass(robot, q);
comFloor = [com(1); com(2); info.floorZ];
end
