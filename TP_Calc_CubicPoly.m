function [pos,spd,acc] = TP_Calc_CubicPoly(TP_CubicPoly,t)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
if(0<=t<=TP_CubicPoly.cycle)
    pos = TP_CubicPoly.coeff(1) + TP_CubicPoly.coeff(2) * t + TP_CubicPoly.coeff(3) * t.^2 + TP_CubicPoly.coeff(4) * t.^3;
    spd = TP_CubicPoly.coeff(2) + 2 * TP_CubicPoly.coeff(3) * t + 3 * TP_CubicPoly.coeff(4) * t.^2;
    acc = 2 * TP_CubicPoly.coeff(3) + 6 * TP_CubicPoly.coeff(4) * t;
else
    disp "Para 't' out of range."
end
end

