package org.firstinspires.ftc.teamcode.pedroPathing.Auto;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Red Goal Right", group = "AutoReal")
public class Red_Goal_Right extends OpMode {

    private Follower follower;
    private int pathState;

    // POSES -------------------------------------------------------------

    private final Pose startPose = new Pose(84.64791901012374, 4.872890888638926, Math.toRadians(90));

    private final Pose shootPose = new Pose(86.91563554555681, 35.514060742407196, Math.toRadians(90));

    private PathChain gotoShootPose;

    public void buildPaths() {
        gotoShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
    }

    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                follower.followPath(gotoShootPose);
                setPathState(-1);
                break;
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();



        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    public void start() {
        setPathState(0);
    }

}