function T = RotY (angle)
C = cos(angle);
S = sin(angle);
T = [ C  0  S; 
      0  1  0;
     -S  0  C];
end