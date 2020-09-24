function [handles] = drawDTL(ax, joint_angles, dtl)
% Input:       ax - handles of app figure axes 
%    joint_angles - 3x1 vector of joint_angles (rad)
%             dtl - robot structure from drawntolifeInit
% Output: handles - 3x1 array of transformations (T1, T2, T3)
% Draws the DTL robot at the joint_angles 

% Create a cell array of DTL forward kinematics transforms
[~,dtl_T] = drawntolifeFK(joint_angles, dtl);

% Define parameters
d1 = dtl.parameters.d1;
a1 = dtl.parameters.l1;
a2 = dtl.parameters.l2;
lpen = dtl.parameters.lpen; %sharpie length

% Plot scaling properties
origin_size = 20;
marker_size = 10;
vector_size = 0.05*max(abs(diff(reshape(dtl.workspace,2,3))));

% Create axes object
xlim(ax, dtl.workspace(1:2));
ylim(ax,dtl.workspace(3:4));
zlim(ax, dtl.workspace(5:6));

view(ax, 3) %default line of sight for 3D
grid(ax, "on");
axis(ax, "equal");
xlabel(ax, 'X (in)','FontSize',12);
ylabel(ax, 'Y (in)','FontSize',12);
zlabel(ax, 'Z (in)','FontSize',12);
title(ax, 'Drawn to Virtual Life', 'Fontsize', 12)

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Draw Robot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create link 0 and frame 0 (base circle)
h = drawRobotFrame([0,0,0]);
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
circ = linspace(0,2*pi,50);
L_0 = line(ax,.7*cos(circ),.7*sin(circ),...
     0*ones(length(circ)),...
    'Color','k','LineWidth',1.5);
set(L_0,'Parent',hg);
T_0 = hgtransform('Parent',ax,'Matrix',eye(4));
set(hg,'Parent',T_0);

% Create link 1 and frame 1
h = drawRobotFrame(dtl.colors{1});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_1 = line(ax,[a1,0, 0], [0,0,0], [0,0,-d1],...
    'Color',dtl.colors{1},'LineWidth',1.5);
set(L_1,'Parent',hg);
T_1 = hgtransform('Parent',T_0,'Matrix',dtl_T{1});
set(hg,'Parent',T_1);

% Create link 2 and frame 2 
h = drawRobotFrame(dtl.colors{2});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_2 = line(ax,[0, a2, a2],[0, 0, 0],[0,0,-lpen],...
    'Color',dtl.colors{2},'LineWidth',1.5);
set(L_2,'Parent',hg);
T_2 = hgtransform('Parent',T_1,'Matrix',dtl_T{2});
set(hg,'Parent',T_2);

% Create link 3 and frame 3
h = drawRobotFrame(dtl.colors{3});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);

T_23 = dtl_T{3};
z_pos = T_23(3,4);
L_3 = line(ax,[0,0], [0,0], [0,-z_pos],...
    'Color',dtl.colors{3},'LineWidth',1.5);
set(L_3,'Parent',hg);
T_3 = hgtransform('Parent',T_2,'Matrix',dtl_T{3});
set(hg,'Parent',T_3);

% Render graphics
set(get(ax,'Parent'),'Renderer','openGL');
drawnow;

% Return hgtransform handles
handles = [T_1,T_2,T_3];

    function h = drawRobotFrame( color )
        % Plot reference frame
        X_b = [vector_size,0,0,1]';
        Y_b = [0,vector_size,0,1]';
        Z_b = [0,0,vector_size,1]';
        h(1) = line(ax, 0,0,0,'Marker','.','MarkerSize',origin_size,'Color',color);
        h(2) = line(ax,[0,X_b(1)],[0,X_b(2)],[0,X_b(3)],'LineWidth',1.5,'Color',color);
        h(3) = line(ax,[0,Y_b(1)],[0,Y_b(2)],[0,Y_b(3)],'LineWidth',1.5,'Color',color);
        h(4) = line(ax,[0,Z_b(1)],[0,Z_b(2)],[0,Z_b(3)],'LineWidth',1.5,'Color',color);
        h(5) = line(ax,X_b(1),X_b(2),X_b(3),'LineWidth',1.5,'Marker','x','MarkerSize',marker_size,'Color',color);
        h(6) = line(ax,Y_b(1),Y_b(2),Y_b(3),'LineWidth',1.5,'Marker','o','MarkerSize',marker_size,'Color',color);
        h(7) = line(ax,Z_b(1),Z_b(2),Z_b(3),'LineWidth',1.5,'Marker','d','MarkerSize',marker_size,'Color',color);
    end

end