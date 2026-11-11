function strategy = chooseStrategy(xi, polyFront, hipExtra)
%CHOOSESTRATEGY  Subtopic 6: pick a recovery strategy from the capture point.
%   ankle : capture point inside the support polygon; moving the CoP with the
%           ankles is enough to stop the CoM
%   hip   : capture point just beyond the front edge; bending at the hip adds
%           up to hipExtra of extra reach
%   step  : capture point too far; the robot must step towards it

if xi <= polyFront
    strategy = "ankle";
elseif xi <= polyFront + hipExtra
    strategy = "hip";
else
    strategy = "step";
end
end
