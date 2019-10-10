function [pos,spd,acc] = TP_Calc_QuinticPoly(TP_QuinticPoly,t)
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here
pos = TP_QuinticPoly.coeff(1) + TP_QuinticPoly.coeff(2)*t + TP_QuinticPoly.coeff(3)*t.^2 + TP_QuinticPoly.coeff(4)*t.^3 + TP_QuinticPoly.coeff(5)*t.^4 + TP_QuinticPoly.coeff(6)*t.^5;
spd = TP_QuinticPoly.coeff(2) + 2*TP_QuinticPoly.coeff(3)*t + 3*TP_QuinticPoly.coeff(4)*t.^2 + 4*TP_QuinticPoly.coeff(5)*t.^3 + 5*TP_QuinticPoly.coeff(6)*t.^4;
acc = 2*TP_QuinticPoly.coeff(3) + 6*TP_QuinticPoly.coeff(4)*t + 12*TP_QuinticPoly.coeff(5)*t.^2 + 20*TP_QuinticPoly.coeff(6)*t.^3;
end

