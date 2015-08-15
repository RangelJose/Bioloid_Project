function Jg=Jg_EE(EE,O,LJoint,parent,axis,Rots)
EtaRow=cell(24,1);
Eta=cell(24,1);
O_f=cell(24,1);
Jv = cell(24,1);
Jw = cell(24,1);
Jg=zeros(6,24);
i=LJoint;

while i>1
        switch axis(i)       
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
        EtaRow{i}=Rots{parent(i)}*Eta{i};
        O_f{i}=EE-O{i}; %Aquisition of On-O's
if axis(i)<=3
    Jv{i}=cross(EtaRow{i},O_f{i});%Aquisition of Jv
        Jw{i}=EtaRow{i};
else
    Jv{i}=EtaRow{i};
        Jw{i}=[0;0;0];
end
Jg(:,i)=[Jv{i};Jw{i}]; %Introducing every line which must go in the Jg
i=parent(i);
end
Jv{1}=[1;0;0];
Jw{1}=[0;0;0];
Jg(:,1)=[Jv{1};Jw{1}];
end