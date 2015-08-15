function T = RotX (angle)
C = cos(angle);
S = sin(angle);
T = [1  0  0;
     0  C -S;
     0  S  C];
end