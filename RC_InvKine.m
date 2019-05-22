function [joint_angle] = RC_InvKine(robot,pos_mat)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
joint_angle = zeros(3,4);
if(strcmp(robot.Para.Mech.type,'elbow-elbow'))
    %LF:
    for i=1:4
        rho = sqrt(pos_mat(1,i)^2+pos_mat(2,i)^2);
        if(i<=2)
            joint_angle(1,i) = atan2(robot.Para.Mech.leg_d/rho,sqrt(1-(robot.Para.Mech.leg_d/rho)^2)) + atan2(pos_mat(2,i),pos_mat(1,i));
            %joint_angle(1,i) = atan2(robot.Para.Mech.leg_d/rho,-sqrt(1-(robot.Para.Mech.leg_d/rho)^2)) + atan2(pos_mat(2,i),pos_mat(1,i));
        else
            joint_angle(1,i) = atan2(-robot.Para.Mech.leg_d/rho,sqrt(1-(robot.Para.Mech.leg_d/rho)^2)) + atan2(pos_mat(2,i),pos_mat(1,i));
            %joint_angle(1,i) = atan2(-robot.Para.Mech.leg_d/rho,-sqrt(1-(robot.Para.Mech.leg_d/rho)^2)) + atan2(pos_mat(2,i),pos_mat(1,i));
        end
        
        C3 = (pos_mat(1,i)^2+pos_mat(2,i)^2+pos_mat(3,i)^2-robot.Para.Mech.leg_a1^2-robot.Para.Mech.leg_a2^2-robot.Para.Mech.leg_d^2)/(2*robot.Para.Mech.leg_a1*robot.Para.Mech.leg_a2)
        if(i<=2)
            S3 = -sqrt(1-C3^2);
        else
            S3 = sqrt(1-C3^2)
        end
        joint_angle(3,i) = atan2(S3,C3);
        
        rho = sqrt((robot.Para.Mech.leg_a1+robot.Para.Mech.leg_a2*C3)^2+(robot.Para.Mech.leg_a2*S3)^2)
        if(i<=2)
            joint_angle(2,i) = atan2(robot.Para.Mech.leg_a1+robot.Para.Mech.leg_a2*C3,robot.Para.Mech.leg_a2*S3) - atan2(sqrt(1-(pos_mat(3,i)/rho)^2),pos_mat(3,i)/rho)
            %joint_angle(2,i) = atan2(robot.Para.Mech.leg_a1+robot.Para.Mech.leg_a2*C3,robot.Para.Mech.leg_a2*S3) - atan2(-sqrt(1-(pos_mat(3,i)/rho)^2),pos_mat(3,i)/rho);
        else
            joint_angle(2,i) = atan2(robot.Para.Mech.leg_a1+robot.Para.Mech.leg_a2*C3,robot.Para.Mech.leg_a2*S3) - atan2(sqrt(1-(pos_mat(3,i)/rho)^2),-pos_mat(3,i)/rho)
            %joint_angle(2,i) = atan2(robot.Para.Mech.leg_a1+robot.Para.Mech.leg_a2*C3,robot.Para.Mech.leg_a2*S3) - atan2(-sqrt(1-(pos_mat(3,i)/rho)^2),-pos_mat(3,i)/rho);
        end
    end
end
end

