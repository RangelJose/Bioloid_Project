function [P,Targets,EE] = SoT()
%% Stack of Tasks
% Outputs:
%    P = Number of priorities
%    Targets = Cell with the targets to accomplish
%    EE = Cell with the end effectors to raise the targets
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Number of priorities
% How Many priorities do you want
P = 4;
EE = cell(P);
%% Targets to accomplish
    %Estructure of targets
    %   Target = [X
    %             Y
    %             Z
    %             E1
    %             E2
    %             E3];
        % Where X,Y and Z are the coordenates for the position to raise with the End-Effector
        % Where E1,E2 and E3 are the Euler angles depend on the convention that the user choose.
%Targets{1} = [0;-0.0385;0;0;0;0];
Targets{1} = [0;-0.1;0;0;0;deg2rad(-10)];
%Targets{2} = [0;0.0385;0;0;0;0];
Targets{2} = [0;0.1;0;0;0;deg2rad(10)];
Targets{3} = [0.05;-0.05;0.3;0;0;0];
%Targets{4} = [0;0;0;0;0;0];
Targets{4} = [0.05;0.05;0.3;0;0;0];
%Targets{5} = [0;0;0;0;0;0];
Targets{5} = [0;0;0;0;deg2rad(10);0];


%% End effector to raise the target
    % Write the end effector to taise the target
        % 'RH' for Right Hand
        % 'LH' for Left Hand
        % 'RF' for Right Foot
        % 'LF' for Left Foot
        % 'C'  for chest
        
EE{1} = 'RF';
EE{2} = 'LF';
EE{3} = 'RH';
EE{4} = 'LH';
EE{5} = 'C' ;

for i=1:P
switch EE{i}
    case 'RH'
        EE{i} = 1;
    case 'LH'
        EE{i} = 2;
    case 'RF'
        EE{i} = 3;
    case 'LF'
        EE{i} = 4;
    case 'C'
        EE{i} = 5;
end
end
end

