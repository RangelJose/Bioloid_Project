function R = Rot(axis, angle)
switch axis
    case 1
        R = RotX3(angle);
    case 2
        R = RotY3(angle);
    case 3
        R = RotZ3(angle);
end
end