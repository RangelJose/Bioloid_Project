function  out = Bioloid_Signals2VRML (EEpose_RH,EEpose_LH,EEpose_RF,EEpose_LF, q)
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
 q6=[0;1;0;q(6)];

 q7=[0;0;-1;q(7)]; %Joints of the right hand
 q8=[0;1;0;q(8)-deg2rad(90)];
 q9=[0;1;0;q(9)];
 
 q10=[0;0;-1;q(10)]; %Joints of the left hand
 q11=[0;1;0;q(11)+deg2rad(90)];
 q12=[0;1;0;q(12)];
 
 q13=[0;1;0;q(13)];%Joints of the right foot
 q14=[1;0;0;q(14)];
 q15=[0;0;-1;q(15)];
 q16=[0;0;-1;q(16)];
 q17=[0;0;-1;q(17)];
 q18=[1;0;0;q(18)];
 
 q19=[0;1;0;q(19)];%Joints of the right foot
 q20=[1;0;0;q(20)];
 q21=[0;0;-1;q(21)];
 q22=[0;0;-1;q(22)];
 q23=[0;0;-1;q(23)];
 q24=[1;0;0;q(24)];
 

EEpose_RH_Tras=[EEpose_RH(1);EEpose_RH(3);-EEpose_RH(2)];
EEpose_RH_X=[1;0;0;EEpose_RH(4)];
EEpose_RH_Y=[0;0;-1;EEpose_RH(5)];
EEpose_RH_Z=[0;1;0;EEpose_RH(6)];

EEpose_LH_Tras=[EEpose_LH(1);EEpose_LH(3);-EEpose_LH(2)];
EEpose_LH_X=[1;0;0;EEpose_LH(4)];
EEpose_LH_Y=[0;0;-1;EEpose_LH(5)];
EEpose_LH_Z=[0;1;0;EEpose_LH(6)];

EEpose_RF_Tras=[EEpose_RF(1);EEpose_RF(3);-EEpose_RF(2)];%Q15 is not working well
EEpose_RF_X=[1;0;0;EEpose_RF(4)];
EEpose_RF_Y=[0;0;-1;EEpose_RF(5)];
EEpose_RF_Z=[0;1;0;EEpose_RF(6)];

EEpose_LF_Tras=[EEpose_LF(1);EEpose_LF(3);-EEpose_LF(2)];
EEpose_LF_X=[1;0;0;EEpose_LF(4)];
EEpose_LF_Y=[0;0;-1;EEpose_LF(5)];
EEpose_LF_Z=[0;1;0;EEpose_LF(6)];

out=[BaseTras;q4;q5;q6;q7;q8;q9;q10;q11;q12;q13;q14;q15;q16;q17;q18;q19;q20;q21;q22;q23;q24;EEpose_RH_Tras;EEpose_RH_X;EEpose_RH_Y;EEpose_RH_Z;EEpose_LH_Tras;EEpose_LH_X;EEpose_LH_Y;EEpose_LH_Z;EEpose_RF_Tras;EEpose_RF_X;EEpose_RF_Y;EEpose_RF_Z;EEpose_LF_Tras;EEpose_LF_X;EEpose_LF_Y;EEpose_LF_Z;];
end
