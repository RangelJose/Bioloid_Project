function [EtaR,Rots] = Eta_Row (nb,qAxes,q,parent)
EtaR=cell(nb,1);
Eta=cell(nb,1);
RotQ=cell(nb,1);
for i=1:nb
    switch qAxes(i)
        case 1
            Eta{i}=[1;0;0];
        case 2
            Eta{i}=[0;1;0];
        case 3
            Eta{i}=[0;0;1];
        case 4
            Eta{i}=[1;0;0];
        case 5
            Eta{i}=[0;1;0];
        case 6
            Eta{i}=[0;0;1];
    end
end %Eta_Hat 

Rots{1}=[1 0 0;
         0 1 0;
         0 0 1];
Rots{2}=[1 0 0;
         0 1 0;
         0 0 1];
Rots{3}=[1 0 0;
         0 1 0;
         0 0 1];
EtaR{1}=Eta{1}; %Eta Row of the first is the same that the Eta_Hat
EtaR{2}=Eta{2};
EtaR{3}=Eta{3};
for i=4:nb
    RotQ{i}=Rot3x3(qAxes(i),q(i));
    EtaR{i}=Rots{parent(i)}*Eta{i};
    Rots{i}=Rots{parent(i)}*RotQ{i};
end %Rotations for every joint from parent

end