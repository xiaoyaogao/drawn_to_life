function setDTL(angles, dtl)
% Update position of DTL robot 
% Input - dtl: robot structure from drawntolifeInit
%      angles: 3x1 vector of desired joint angles (rad) used to set robot

[~,dtl_T] = drawntolifeFK(angles,dtl);
set(dtl.handles(1),'Matrix',dtl_T{1});
set(dtl.handles(2),'Matrix',dtl_T{2});
set(dtl.handles(3),'Matrix',dtl_T{3});
drawnow;
end