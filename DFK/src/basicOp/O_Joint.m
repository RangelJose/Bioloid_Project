function [O,Rots]=O_Joint(nb,T)
O=cell(nb,1);
for i=1:nb
O{i}=T{i}(1:3,4);
Rots{i}=T{i}(1:3,1:3);
end
end