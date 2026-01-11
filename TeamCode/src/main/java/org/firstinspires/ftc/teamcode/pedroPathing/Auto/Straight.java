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
@Autonomous(name = "Striaght Auto", group = "Examples")
public class Straight extends OpMode {

    private Follower follower;
    private int pathState;

    // POSES -------------------------------------------------------------

    private final Pose startPose = new Pose(79.73094170403586, 8.6457399103139, Math.toRadians(90));

    private final Pose shootPose = new Pose(79.85201793721973, 35.64125560538115, Math.toRadians(90));

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