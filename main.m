clc
clear
close all
Robot = RC_Init_Robot('elbow-elbow',70,300,200,400,600);

AXIS_LEN = 180;
AXIS_THICK = 2;

Robot.Leg.LF.SerialLink.plot([0 0 -pi/2]);
hold on;
Robot.Leg.LH.SerialLink.plot([0 0 -pi/2]);
Robot.Leg.RF.SerialLink.plot([0 0 pi/2]);
Robot.Leg.RH.SerialLink.plot([0 0 pi/2]);

Robot.Leg.LF.BaseMat.trplot('length',AXIS_LEN,'thick',AXIS_THICK,'frame','LF','rgb');
Robot.Leg.LH.BaseMat.trplot('length',AXIS_LEN,'thick',AXIS_THICK,'frame','LH','rgb');
Robot.Leg.RF.BaseMat.trplot('length',AXIS_LEN,'thick',AXIS_THICK,'frame','RF','rgb');
Robot.Leg.RH.BaseMat.trplot('length',AXIS_LEN,'thick',AXIS_THICK,'frame','RH','rgb');

plot_box('centre',[0 0],'size',[600 400],'fillcolor','b','alpha',0.3) 

pos = [ 350 320 320 320
        -70  -70   70   70
        0   0   0   0   ];

body_xyz = [    0       0       400]
body_rpy = [    0       -pi/12    	0]
plain_rp = [    0       -pi/12]

for phase=0:0.01:1
    [Robot,pos,BodyFrm2p] = RC_MovCtrl(Robot,phase,'trot',1.0,0.5,0,0,100,0,body_xyz(1),body_xyz(2),body_xyz(3),body_rpy(1),body_rpy(2),body_rpy(3),plain_rp(1),plain_rp(2),400,600,0,0);
    res = RC_InvKine(Robot,pos);
    Robot.Leg.LF.SerialLink.plot(res(:,1)');
    Robot.Leg.LH.SerialLink.plot(res(:,2)');
    Robot.Leg.RF.SerialLink.plot(res(:,3)');
    Robot.Leg.RH.SerialLink.plot(res(:,4)');
    plot3(BodyFrm2p.LF.t(1),BodyFrm2p.LF.t(2),BodyFrm2p.LF.t(3),'ro');
    plot3(BodyFrm2p.LH.t(1),BodyFrm2p.LH.t(2),BodyFrm2p.LH.t(3),'ro');
    plot3(BodyFrm2p.RF.t(1),BodyFrm2p.RF.t(2),BodyFrm2p.RF.t(3),'ro');
    plot3(BodyFrm2p.RH.t(1),BodyFrm2p.RH.t(2),BodyFrm2p.RH.t(3),'ro');
    drawnow
end


