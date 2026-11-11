function xi = capturePoint(x, v, omega)
%CAPTUREPOINT  Subtopic 5: where the robot must put its CoP (or foot) to stop.
%   xi = x + v / omega,   omega = sqrt(g / z0)

xi = x + v ./ omega;
end
