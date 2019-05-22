 function [Robot] = RC_Init_Robot(type,offset,length1,length2,body_width,body_length)
%CreateRobot Create a quadruped robot object
%   ‘robot’ object is created with Standard DH method. 
%   The explanation of function para are as following:
%       type - configuation structure of quadruped robot
%       offset, length1, length2 - parameters of legs
%       body_width - the width between No.0 leg coordinates
%       body_length - the length between No.0 leg coordinates

Robot.Para.Mech.type = type;
Robot.Para.Mech.leg_d = offset;
Robot.Para.Mech.leg_a1 = length1;
Robot.Para.Mech.leg_a2 = length2;
Robot.Para.Mech.body_w = body_width;
Robot.Para.Mech.body_l = body_length;

if(strcmp(type,'elbow-elbow'))
    Robot.Leg.LF.Link(1) = Link('d',0,'a',0,'alpha',pi/2,'standard');
    Robot.Leg.LF.Link(2) = Link('d',offset,'a',length1,'alpha',0,'standard');
    Robot.Leg.LF.Link(3) = Link('d',0,'a',length2,'alpha',0,'standard');
    Robot.Leg.LF.BaseMat = SE3(body_length/2, body_width/2, 0) * SE3.Ry(-pi/2) * SE3.Rz(pi);
    Robot.Leg.LF.SerialLink = SerialLink(Robot.Leg.LF.Link,'name','LF','base',Robot.Leg.LF.BaseMat); 
    
    Robot.Leg.LH.Link(1) = Link('d',0,'a',0,'alpha',pi/2,'standard');
    Robot.Leg.LH.Link(2) = Link('d',offset,'a',length1,'alpha',0,'standard');
    Robot.Leg.LH.Link(3) = Link('d',0,'a',length2,'alpha',0,'standard');
    Robot.Leg.LH.BaseMat = SE3(-body_length/2, body_width/2, 0) * SE3.Ry(-pi/2) * SE3.Rz(pi);
    Robot.Leg.LH.SerialLink = SerialLink(Robot.Leg.LH.Link,'name','LH','base',Robot.Leg.LH.BaseMat); 

    Robot.Leg.RF.Link(1) = Link('d',0,'a',0,'alpha',-pi/2,'standard');
    Robot.Leg.RF.Link(2) = Link('d',offset,'a',length1,'alpha',0,'standard');
    Robot.Leg.RF.Link(3) = Link('d',0,'a',length2,'alpha',0,'standard');
    Robot.Leg.RF.BaseMat = SE3(body_length/2, -body_width/2, 0) * SE3.Ry(-pi/2) * SE3.Rz(pi);
    Robot.Leg.RF.SerialLink = SerialLink(Robot.Leg.RF.Link,'name','RF','base',Robot.Leg.RF.BaseMat); 
    
    Robot.Leg.RH.Link(1) = Link('d',0,'a',0,'alpha',-pi/2,'standard');
    Robot.Leg.RH.Link(2) = Link('d',offset,'a',length1,'alpha',0,'standard');
    Robot.Leg.RH.Link(3) = Link('d',0,'a',length2,'alpha',0,'standard');
    Robot.Leg.RH.BaseMat = SE3(-body_length/2, -body_width/2, 0) * SE3.Ry(-pi/2) * SE3.Rz(pi);
    Robot.Leg.RH.SerialLink = SerialLink(Robot.Leg.RH.Link,'name','RH','base',Robot.Leg.RH.BaseMat); 
end

end

