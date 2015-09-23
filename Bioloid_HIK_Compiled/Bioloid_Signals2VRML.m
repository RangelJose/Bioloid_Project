function  out = Bioloid_Signals2VRML(q)
%% Signals to VRML
% This file controls the signals to send to VRML(Virtual Model)
% Inputs:
%     q=Robot configuration, i.e. each element of q is a joint position
% 
% Outputs:
%     VRML signals= This send the Virtual Model signals to control the position of the frame of each end-effector    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[~,Targets,~] = SoT();

BaseTras=[q(1);q(3);-q(2)];%Joints of the Inertial to the base
 q4=[1;0;0;q(4)];
 q5=[0;0;-1;q(5)];
 q6=[0;1;0;q(6)];           %15 outputs

 q7=[0;0;-1;q(7)]; %Joints of the right hand
 q8=[0;1;0;q(8)-deg2rad(90)];
 q9=[0;1;0;q(9)];           %12 outputs
 
 q10=[0;0;-1;q(10)]; %Joints of the left hand
 q11=[0;1;0;q(11)+deg2rad(90)];
 q12=[0;1;0;q(12)];         %12 outputs
 
 q13=[0;1;0;q(13)];%Joints of the right foot
 q14=[1;0;0;q(14)];
 q15=[0;0;-1;q(15)];
 q16=[0;0;-1;q(16)];
 q17=[0;0;-1;q(17)];
 q18=[1;0;0;q(18)];     %24 outputs
 
 q19=[0;1;0;q(19)];%Joints of the right foot
 q20=[1;0;0;q(20)];
 q21=[0;0;-1;q(21)];
 q22=[0;0;-1;q(22)];
 q23=[0;0;-1;q(23)];
 q24=[1;0;0;q(24)];     %24 outputs

 for i=1:5
  Tras_Obj = [Targets{i}(1);Targets{i}(3);-Targets{i}(2)];
  Rot_Obj = Euler_ToVRML(1,Targets{i}(4),Targets{i}(5),Targets{i}(6));
  Obj{i} = [Tras_Obj;Rot_Obj];
 end
 
out=[BaseTras;q4;q5;q6;q7;q8;q9;q10;q11;q12;q13;q14;q15;q16;q17;q18;q19;q20;q21;q22;q23;q24;Obj{1};Obj{2};Obj{3};Obj{4};Obj{5}];
end
