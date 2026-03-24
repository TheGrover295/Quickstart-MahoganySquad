package org.firstinspires.ftc.teamcode.pedroPathing.AutoRecord;

import com.pedropathing.geometry.Pose;

public class ReplayPID {
    AutoReplayAllenTest autoReplay;

    double integralHistoryX, integralHistoryY, integralHistoryTheta, previousTime, previousErrorTheta, previousErrorY, previousErrorX ;

    public ReplayPID(AutoReplayAllenTest autoReplay){
        this.autoReplay = autoReplay;
    }

    public double[] replayPIDMotorValues(double currentTime, Pose currentPose, double endTime){
        if (currentTime >= endTime - .01) {return null;}
        double[] targetList;
        targetList = autoReplay.getTargets(currentTime); //xTarget 0, yTarget 1, headingTarget 2, vxTarget 3, vyTarget 4, omegaTargeta 5, xTarget 6, ayTarget 7, alphaTarget 8\

        // first for the feed forward system that will predict motor powers to reach target acceleration and velocity
        double kVxy = .0; //random needs tuning
        double kAxy = 0; //acceleration
        double kVtheta = .0; //random needs tuning
        double kAtheta = 0; //acceleration
        double ffX = kVxy * targetList[3] + kAxy * targetList[6];
        double ffY = kVxy * targetList[4] + kAxy * targetList[7];
        double ffTheta = kVtheta * targetList[5] + kAtheta * targetList[8];

        double[] errorList = autoReplay.getError(currentTime, currentPose);
        double dT = currentTime - previousTime;
        if (dT <= 0) dT = 0.01;
        
        double kP = .1, kI = 0.0, kD = 0.01;
        double kPtheta = .1, kItheta = 0.0, kDtheta = 0.01;
        double pidX = kP * errorList[0] + kI * integralHistoryX + kD * (errorList[0] - previousErrorX) / dT;
        double pidY = kP * errorList[1] + kI * integralHistoryY + kD * (errorList[1] - previousErrorY) /dT;
        double pidTheta = kPtheta * errorList[2] + kItheta * integralHistoryTheta + kDtheta * (errorList[2] - previousErrorTheta) / dT;


        double controlX = ffX +pidX;
        double controlY = ffY +pidY;
        double controlTheta = ffTheta +pidTheta;

        double cosH = Math.cos(Math.PI/2 - currentPose.getHeading());
        double sinH = Math.sin(Math.PI/2 - currentPose.getHeading());

        double forward = controlX * sinH + controlY * cosH; // robot-forward
        double strafe  = controlX * cosH - controlY * sinH; // robot-left

        double fl = forward + strafe + controlTheta;
        double fr = forward - strafe - controlTheta;
        double bl = forward - strafe + controlTheta;
        double br = forward + strafe - controlTheta;

        //normalize to one
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        integralHistoryX += errorList[0] * dT;
        integralHistoryY += errorList[1] * dT;
        integralHistoryTheta += errorList[2] * dT;
        previousTime = currentTime;
        previousErrorTheta = errorList[2];
        previousErrorY = errorList[1];
        previousErrorX = errorList[0];

        return new double[]{fl, fr, bl, br};
    }
}
