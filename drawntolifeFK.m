function [T, drawntolife_T] = drawntolifeFK(joints, drawntolife)
% Inputs: joints - 3x1 vector of current joint angles (rad)     
%    drawntolife - dtl structure from drawntolifeInit
% Outputs:     T - homogeneous transformation matrix from frame0 to frame3
%  drawntolife_T - 3x1 cell array of T01, T12, T23

% Check that current joint angles are within joint limits
for i = 1:length(joints)
    if drawntolife.limits{i}(1) > joints(i)...
            || drawntolife.limits{i}(2) < joints(i)
        warning(['Joint specification',num2str(i),' outside of limits.']);
    end
end

%joints contains theta1, theta2, and d 
%where d is the distance downwards we want the pen to move
T0_1 = dhtf(0, 0, drawntolife.parameters.d1, joints(1));
T1_2 = dhtf(0, drawntolife.parameters.l1, 0, joints(2)-(pi/2)); % home configuration is elbow up
T2_3 = dhtf(0, drawntolife.parameters.l2,...
    -drawntolife.parameters.lpen+joints(3), 0);

% Output calculated transformation matrices
drawntolife_T = {T0_1,T1_2,T2_3};
T = T0_1*T1_2*T2_3;
end