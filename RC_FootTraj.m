function [pos,yaw] = RC_FootTraj(robot,phase)
%UNTITLED14 Summary of this function goes here
%   Detailed explanation goes here

span_x = robot.Para.Mov.span_x;
span_y = robot.Para.Mov.span_y;
span_z = robot.Para.Mov.span_z;
spd_x = robot.Para.Mov.span_x/robot.Para.Mov.dutyratio;
spd_y = robot.Para.Mov.span_y/robot.Para.Mov.dutyratio;

DirX_Seg(1) = TP_Init_QuinticPoly(-span_x/2, -spd_x, 0, 0, 2*spd_x, 0, (1-robot.Para.Mov.dutyratio)/2);
DirX_Seg(2) = TP_Init_QuinticPoly(0, 2*spd_x, 0, span_x/2, -spd_x, 0, (1-robot.Para.Mov.dutyratio)/2);
DirX_Seg(3) = TP_Init_QuinticPoly(span_x/2, -spd_x, 0, -span_x/2, -spd_x, 0, robot.Para.Mov.dutyratio);
DirY_Seg(1) = TP_Init_QuinticPoly(-span_y/2, -spd_y, 0, 0, 2*spd_y, 0, (1-robot.Para.Mov.dutyratio)/2);
DirY_Seg(2) = TP_Init_QuinticPoly(0, 2*spd_y, 0, span_y/2, -spd_y, 0, (1-robot.Para.Mov.dutyratio)/2);
DirY_Seg(3) = TP_Init_QuinticPoly(span_y/2, -spd_y, 0, -span_y/2, -spd_y, 0, robot.Para.Mov.dutyratio);
DirZ_Seg(1) = TP_Init_QuinticPoly(0, 0, 0, span_z, 0, 0, (1-robot.Para.Mov.dutyratio)/2);
DirZ_Seg(2) = TP_Init_QuinticPoly(span_z, 0, 0, 0, 0, 0, (1-robot.Para.Mov.dutyratio)/2);
DirZ_Seg(3) = TP_Init_QuinticPoly(0, 0, 0, 0, 0, 0, robot.Para.Mov.dutyratio);

if(0<=phase && phase<=(1-robot.Para.Mov.dutyratio)/2)
    pos(1) = TP_Calc_QuinticPoly(DirX_Seg(1),phase);
    pos(2) = TP_Calc_QuinticPoly(DirY_Seg(1),phase);
    pos(3) = TP_Calc_QuinticPoly(DirZ_Seg(1),phase);
elseif((1-robot.Para.Mov.dutyratio)/2<phase && phase<=1-robot.Para.Mov.dutyratio)
    pos(1) = TP_Calc_QuinticPoly(DirX_Seg(2),phase-(1-robot.Para.Mov.dutyratio)/2);
    pos(2) = TP_Calc_QuinticPoly(DirY_Seg(2),phase-(1-robot.Para.Mov.dutyratio)/2);
    pos(3) = TP_Calc_QuinticPoly(DirZ_Seg(2),phase-(1-robot.Para.Mov.dutyratio)/2);
elseif(1-robot.Para.Mov.dutyratio<phase && phase<=1.0)
    pos(1) = TP_Calc_QuinticPoly(DirX_Seg(3),phase-(1-robot.Para.Mov.dutyratio));
    pos(2) = TP_Calc_QuinticPoly(DirY_Seg(3),phase-(1-robot.Para.Mov.dutyratio));
    pos(3) = TP_Calc_QuinticPoly(DirZ_Seg(3),phase-(1-robot.Para.Mov.dutyratio));
end

end

