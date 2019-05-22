function [robot,pos,BodyFrm2p] = RC_MovCtrl(robot,phase_,gait,cycle,dutyratio,span_x,span_y,span_z,span_w,body_x,body_y,body_z,body_roll,body_pitch,body_yaw,pos_roll,pos_pitch,zero_w,zero_l,zero_x,zero_y)
%UNTITLED15 Summary of this function goes here
%   Detailed explanation goes here
robot.Para.Mov.gait = gait;
robot.Para.Mov.span_x = span_x;
robot.Para.Mov.span_y = span_y;
robot.Para.Mov.span_z = span_z;
robot.Para.Mov.span_w = span_w;
robot.Para.Mov.dutyratio = dutyratio; 
robot.Para.Mov.cycle = cycle;
 
robot.Para.Pose.body_x = body_x;
robot.Para.Pose.body_y = body_y;
robot.Para.Pose.body_z = body_z;
robot.Para.Pose.body_roll = body_roll;
robot.Para.Pose.body_pitch = body_pitch;
robot.Para.Pose.body_yaw = body_yaw;
robot.Para.Pose.pos_roll = pos_roll;
robot.Para.Pose.pos_pitch = pos_pitch;

robot.Para.Zero.width = zero_w;
robot.Para.Zero.length = zero_l;
robot.Para.Zero.centre_x = zero_x;
robot.Para.Zero.centre_y = zero_y;

RefFrm2BodyFrm = SE3(body_x,body_y,body_z) * SE3.Rx(body_roll) * SE3.Ry(body_pitch) * SE3.Rz(body_yaw);
RefFrm2PosFrm = SE3.Rx(pos_roll) * SE3.Ry(pos_pitch);
BodyFrm2LegFrm.LF = robot.Leg.LF.BaseMat;
BodyFrm2LegFrm.LH = robot.Leg.LH.BaseMat;
BodyFrm2LegFrm.RF = robot.Leg.RF.BaseMat;
BodyFrm2LegFrm.RH = robot.Leg.RH.BaseMat;
PosFrm2ZeroFrm.LF = SE3(zero_x+zero_l/2,zero_y+zero_w/2,0);
PosFrm2ZeroFrm.LH = SE3(zero_x-zero_l/2,zero_y+zero_w/2,0);
PosFrm2ZeroFrm.RF = SE3(zero_x+zero_l/2,zero_y-zero_w/2,0);
PosFrm2ZeroFrm.RH = SE3(zero_x-zero_l/2,zero_y-zero_w/2,0);

if(0<=phase_ && phase_<0.5)
    DirW_Seg = TP_Init_QuinticPoly(0, 0, 0, span_w, 0, 0, 0.5);
    omega = TP_Calc_QuinticPoly(DirW_Seg,phase_);
elseif(0.5<=phase_ && phase_<=1)
    DirW_Seg = TP_Init_QuinticPoly(span_w, 0, 0, 0, 0, 0, 0.5);
    omega = TP_Calc_QuinticPoly(DirW_Seg,phase_-0.5);
end

if(strcmp(gait,'trot'))
    phase(1) = phase_;          %LF
    phase(2) = phase_ + 0.5;    %LH
    phase(3) = phase_ + 0.5;    %RF
    phase(4) = phase_;          %RH
end
for idx = 1:4
    if(phase(idx)>=1)
        phase(idx)=phase(idx)-1;
    end
end

ZeroFrm2p.LF = SE3(RC_FootTraj(robot,phase(1)));
ZeroFrm2p.LH = SE3(RC_FootTraj(robot,phase(2)));
ZeroFrm2p.RF = SE3(RC_FootTraj(robot,phase(3)));
ZeroFrm2p.RH = SE3(RC_FootTraj(robot,phase(4)));
PosFrm2p.LF = SE3.Rz(omega) * PosFrm2ZeroFrm.LF * ZeroFrm2p.LF;
PosFrm2p.LH = SE3.Rz(omega) * PosFrm2ZeroFrm.LH * ZeroFrm2p.LH;
PosFrm2p.RF = SE3.Rz(omega) * PosFrm2ZeroFrm.RF * ZeroFrm2p.RF;
PosFrm2p.RH = SE3.Rz(omega) * PosFrm2ZeroFrm.RH * ZeroFrm2p.RH;
RefFrm2p.LF = RefFrm2PosFrm * PosFrm2p.LF;
RefFrm2p.LH = RefFrm2PosFrm * PosFrm2p.LH;
RefFrm2p.RF = RefFrm2PosFrm * PosFrm2p.RF;
RefFrm2p.RH = RefFrm2PosFrm * PosFrm2p.RH;
BodyFrm2p.LF = InvHomoMat(RefFrm2BodyFrm) * RefFrm2p.LF;
BodyFrm2p.LH = InvHomoMat(RefFrm2BodyFrm) * RefFrm2p.LH;
BodyFrm2p.RF = InvHomoMat(RefFrm2BodyFrm) * RefFrm2p.RF;
BodyFrm2p.RH = InvHomoMat(RefFrm2BodyFrm) * RefFrm2p.RH;
LegFrm2p.LF = InvHomoMat(BodyFrm2LegFrm.LF) * BodyFrm2p.LF;
LegFrm2p.LH = InvHomoMat(BodyFrm2LegFrm.LH) * BodyFrm2p.LH;
LegFrm2p.RF = InvHomoMat(BodyFrm2LegFrm.RF) * BodyFrm2p.RF;
LegFrm2p.RH = InvHomoMat(BodyFrm2LegFrm.RH) * BodyFrm2p.RH;

pos = [LegFrm2p.LF.t LegFrm2p.LH.t LegFrm2p.RF.t LegFrm2p.RH.t];

end

