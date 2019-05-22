function [T] = StdDHMat(theta,d,a,alpha)
%DHStandardMatrix Calc the transform matrix of standard DH method
%   The return val is SE3()
T = [   cos(theta)     -sin(theta)*cos(alpha)      sin(theta)*sin(alpha)   a*cos(theta)
        sin(theta)     cos(theta)*cos(alpha)      -cos(theta)*sin(alpha)   a*sin(theta)
            0               sin(alpha)                  cos(alpha)          d
            0                   0                           0               1           ];
T = SE3(T);
end

