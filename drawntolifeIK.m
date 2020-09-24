function [is_sol, joints] = drawntolifeIK(point, prev_joints, drawntolife)
d1 = drawntolife.parameters.d1;
l1 = drawntolife.parameters.l1;
l2 = drawntolife.parameters.l2;
lpen = drawntolife.parameters.lpen;

% End effector position
x = point(1);
y = point(2);
z = point(3);

beta = acos((l1^2+x^2+y^2-l2^2)/(2*l1*sqrt(x^2+y^2)));
theta1 = [atan2(y,x)-beta;atan2(-y,-x)-beta;atan2(y,x)+beta;atan2(-y,-x)+beta;];
theta2 = atan2(y-l1*sin(theta1),x-l1*cos(theta1))-theta1+(pi/2);
potentialthetas = [theta1, theta2];

% Check joint limits
bad_sol = false(size(potentialthetas,1),1);
for i = 1:size(potentialthetas,1)
    for j = 1:size(potentialthetas,2)
        while potentialthetas(i,j) < drawntolife.limits{j}(1)
            potentialthetas(i,j) = potentialthetas(i,j) + 2*pi;
        end
        while potentialthetas(i,j) > drawntolife.limits{j}(2)
            potentialthetas(i,j) = potentialthetas(i,j) - 2*pi;
        end
        if potentialthetas(i,j) < drawntolife.limits{j}(1)
            bad_sol(i) = true;
        end
    end
end
potentialthetas(bad_sol,:) = []; % remove solutions outside of joint ranges

% Check for solutions, and find "nearest" solution
if isempty(potentialthetas) 
    is_sol = false;
    joints = [];
    warning('no solution');
else
    is_sol = true;
    i = 1;
    while size(potentialthetas,1) > 1
        theta_diff = abs(prev_joints(i)-potentialthetas(:,i));
        potentialthetas = potentialthetas(theta_diff==min(theta_diff),:);
        i = i + 1;
        if i > size(potentialthetas,2)
            potentialthetas = potentialthetas(1,:);
        end
    end
    joints = potentialthetas(1,:);
end       
d = -(prev_joints(3) - z);
if d < drawntolife.limits{3}(1) || d > drawntolife.limits{3}(2)
    is_sol = false;
    d = NaN;
    warning('joint 3 error: the pen cannot move this distance');
end
joints = [potentialthetas d];
end
