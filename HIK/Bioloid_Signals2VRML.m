function  out = Bioloid_Signals2VRML(q,task1,task2,task3,task4,task5,Index)
%% Signals to VRML
% This file controls the signals to send to VRML(Virtual Model)
% Inputs:
%     q=Robot configuration, i.e. each element of q is a joint position
%     EEpose_RH= Right Hand pose Xi of the Right Hand
%     EEpose_LH= Left Hand pose Xi of the Left Hand
%     EEpose_RF= Right Foot pose Xi of the Right Foot
%     EEpose_LF= Left Foot pose Xi of the Left Foot
% 
% Outputs:
%     VRML signals= This send the Virtual Model signals to control the position of the frame of each end-effector    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

 
Obj1_Tras = [task1(1);task1(3);-task1(2)];
Obj1_Ori = Euler_ToVRML(Index,task1(4),task1(5),task1(6)); %15 outputs

Obj2_Tras = [task2(1);task2(3);-task2(2)];
Obj2_Ori = Euler_ToVRML(Index,task2(4),task2(5),task2(6)); %15 outputs

Obj3_Tras = [task3(1);task3(3);-task3(2)];
Obj3_Ori = Euler_ToVRML(Index,task3(4),task3(5),task3(6)); %15 outputs

Obj4_Tras = [task4(1);task4(3);-task4(2)];
Obj4_Ori = Euler_ToVRML(Index,task4(4),task4(5),task4(6)); %15 outputs

Obj5_Tras = [task5(1);task5(3);-task5(2)];
Obj5_Ori = Euler_ToVRML(Index,task5(4),task5(5),task5(6)); %15 outputs

out=[BaseTras;q4;q5;q6;q7;q8;q9;q10;q11;q12;q13;q14;q15;q16;q17;q18;q19;q20;q21;q22;q23;q24;Obj1_Tras;Obj1_Ori;Obj2_Tras;Obj2_Ori;Obj3_Tras;Obj3_Ori;Obj4_Tras;Obj4_Ori;Obj5_Tras;Obj5_Ori];


end
