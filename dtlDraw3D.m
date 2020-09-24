function dtlDraw3D( app )
% MECH 498: Drawn to Virtual Life
% Input: app - handles of Appdesigner application
% Simulates robot drawing inside app figure

% Initialize drawing space and set paper dimensions
close all
im_file = app.im_file;
ax = app.UIAxes;

% Set dimensions and locations of virtual paper wrt the robot frame
paper_dim = [11 8.5];
paper_orig = [2.5 -6];

% Initialize robot and image path
dtl = drawntolifeInit();
[s, c] = get_image_path(im_file, paper_dim, paper_orig, dtl);
prev_joints = zeros(1,3);
dtl.handles = drawDTL(ax, prev_joints,dtl);
hold(ax,"on");

% Iterate through all points on drawing path and set robot to new position
for t = 1:size(s,1)
    % Set robot to next path position if IK solution exists
    [is_sol,joints] = drawntolifeIK(s(t,:),prev_joints,dtl);
    joints(3) = prev_joints(3) + joints(3);%account for fact that d is change in pos, not absolute pos
    if is_sol == 1
        setDTL(joints, dtl);
    else
        disp('This position and orientation is not possible.');
        return
    end
    
    % Plot point if solution exists and point is not on a transition path
    % (c = 0) but an actual image point
    if is_sol == 1 && c(t) ~= 0
        plot3(ax,s(t,1), s(t,2), s(t,3), 'Marker','.');
    end   
    
    % Set current joints as previous joints for next iteration
    prev_joints = joints;
end
end