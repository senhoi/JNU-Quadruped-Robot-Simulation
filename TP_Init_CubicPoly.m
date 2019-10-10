function [TP_CubicPoly] = TP_Init_CubicPoly(init_pos, init_spd, final_pos, final_spd, cycle)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
TP_CubicPoly.cycle = cycle;
TP_CubicPoly.init_pos = init_pos;
TP_CubicPoly.init_spd = init_spd;
TP_CubicPoly.final_pos = final_pos;
TP_CubicPoly.final_spd = final_spd;
TP_CubicPoly.coeff(1) = init_pos;
TP_CubicPoly.coeff(2) = init_spd;
TP_CubicPoly.coeff(3) = 3/cycle^2*(final_pos-init_pos)-2/cycle*init_spd-1/cycle*final_spd;
TP_CubicPoly.coeff(4) = -2/cycle^3*(final_pos-init_pos)+1/cycle^2*(init_spd+final_spd);
end

