function [ drawntolife_struct ] = drawntolifeInit()
% Input: n/a
% Output: drawntolife_struct - structure containing intialized robot
% parameters

d1 = 3; %height of robot base [in]
l1 = 5.5; % length of link 1 [in]
l2 = 5.5; % length of link 2 [in]
lpen = 2.5; %length of pen below gripper [in]
drawntolife_struct.parameters.d1 = d1;
drawntolife_struct.parameters.l1 = l1;
drawntolife_struct.parameters.l2 = l2;
drawntolife_struct.parameters.lpen = lpen;


% Robot joint limits (deg)
deg2rad = pi/180;
drawntolife_struct.limits{1} = [-180,180]*deg2rad;
drawntolife_struct.limits{2} = [-180,180]*deg2rad;
drawntolife_struct.limits{3} = [-(d1-lpen),1]; %[in]

% Set bounds on the cartesian workspace of the robot
drawntolife_struct.workspace = [-(l1+l2),(l1+l2),-(l1+l2),(l1+l2),0,d1-lpen+1]; %+1 to allow for some retraction
% Colors!
drawntolife_struct.colors = default_colors(4);

    function [ C ] = default_colors( n_colors )
    % Outputs the default colors for MATLAB 2015
    C = {[     0,    0.4470,    0.7410];
         [0.8500,    0.3250,    0.0980];
         [0.9290,    0.6940,    0.1250];
         [0.4940,    0.1840,    0.5560];
         [0.4660,    0.6740,    0.1880];
         [0.3010,    0.7450,    0.9330];
         [0.6350,    0.0780,    0.1840]};
    C = C(1:n_colors);
    end
end
