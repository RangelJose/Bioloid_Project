function  out = ToSimulink_Bioloid_FK_ST (q)
%% Forward kinematics by means Successive Transformations
% Inputs: 
%    q = Robot configuration, i.e. each element of q is a joint position
%
% Outputs: 
%    T_ee(1:3,1:3) = End-effector rotation matrix 
%    T_ee(1:3,4) = End-effector position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global srcLoaded Bioloidmodel
if isempty(Bioloidmodel)
    if isempty(srcLoaded)
        addpath(genpath('src'));
        display('--> Folder src and subfolders added to the path')
        srcLoaded = true;
    end
    Bioloid_Model();
    display('--> Bioloid model loaded')
end

[~, T_ee] = Bioloid_T (q);

E_RH=EulerXYZ(T_ee{1}(1:3,1:3));
E_LH=EulerXYZ(T_ee{2}(1:3,1:3));
E_RF=EulerXYZ(T_ee{3}(1:3,1:3));
E_LF=EulerXYZ(T_ee{4}(1:3,1:3));
out = [T_ee{1}(1:3,4);E_RH.';T_ee{2}(1:3,4);E_LH.';T_ee{3}(1:3,4);E_RF.';T_ee{4}(1:3,4);E_LF.'];

end
