function Jw = Jw_Joint (nb,qAxes,EtaRow)

Jw=cell(nb,1);
for i=1:nb
    switch qAxes(i)
        case 1
            Jw{i}=EtaRow{i};
        case 2
            Jw{i}=EtaRow{i};
        case 3
            Jw{i}=EtaRow{i};
        case 4
            Jw{i}=[0;0;0];
        case 5
            Jw{i}=[0;0;0];
        case 6
            Jw{i}=[0;0;0];
    end
end
end