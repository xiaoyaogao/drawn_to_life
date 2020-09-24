function T = dhtf(alpha, a, d, theta)
%{
    Input: DH parameters defined as follows
        alpha - angle between previous and current z, along previous x
        a     - distance between previous and current z, along previous x
        d     - distance between previous and current x, along current z
        theta - angle between previous and current x, along current z
    All angles in radians
    Output: T - homogeneous transform matrix according to DH convention
%}
T = [cos(theta) -sin(theta) 0 a;
    sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d;
    sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d;
    0 0 0 1];
end

