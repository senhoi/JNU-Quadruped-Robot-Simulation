function [Res] = InvHomoMat(T)
%InvHomoMat Inverse matrix calculation for the homogeneous transformation
%   Attention: The function is only suitable for the homogeneous
%   transformation.
R = T.R;
P = T.t;
P = -R.'*P;
R = R.';
Res = SE3([R,P;[0 0 0 1]]);
end

