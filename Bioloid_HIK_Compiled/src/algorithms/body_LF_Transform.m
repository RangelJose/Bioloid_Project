function [T,O,Rots] = body_LF_Transform (nb, bodyLFinPF, qAxes, q, parent)
%Each element in T{i} is the transformation matrix from the inertial frame to
%the beginning of body i after the variation of q(i)
T = cell(nb,1);
O = cell(nb,1);
Rots = cell(nb,1);

T{1} = bodyLFinPF{1}*HT(qAxes(1), q(1));
O{1}=T{1}(1:3,4);
Rots{1}=T{1}(1:3,1:3);
for i = 2:nb
    T{i} = T{parent(i)}*bodyLFinPF{i}*HT(qAxes(i), q(i));
    O{i}=T{i}(1:3,4);
    Rots{i}=T{i}(1:3,1:3);
end
end