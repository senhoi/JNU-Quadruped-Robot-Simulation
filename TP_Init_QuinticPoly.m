function [TP_QuinticPoly] = TP_Init_QuinticPoly(init_pos, init_spd, init_acc, final_pos, final_spd, final_acc, cycle)
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
TP_QuinticPoly.cycle = cycle;
TP_QuinticPoly.init_pos = init_pos;
TP_QuinticPoly.init_spd = init_spd;
TP_QuinticPoly.init_acc = init_acc;
TP_QuinticPoly.final_pos = final_pos;
TP_QuinticPoly.final_spd = final_spd;
TP_QuinticPoly.final_acc = final_acc;
TP_QuinticPoly.coeff(1) = init_pos;
TP_QuinticPoly.coeff(2) = init_spd;
TP_QuinticPoly.coeff(3) = init_acc/2;
TP_QuinticPoly.coeff(4) = (20*final_pos-20*init_pos-(8*final_spd+12*init_spd)*cycle-(3*init_acc-final_acc)*cycle^2)/(2*cycle^3);
TP_QuinticPoly.coeff(5) = (-30*final_pos+30*init_pos+(14*final_spd+16*init_spd)*cycle+(3*init_acc-2*final_acc)*cycle^2)/(2*cycle^4);
TP_QuinticPoly.coeff(6) = (12*final_pos-12*init_pos-(6*final_spd+6*init_spd)*cycle-(init_acc-final_acc)*cycle^2)/(2*cycle^5);
end

