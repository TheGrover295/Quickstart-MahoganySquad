package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Blue Goal Close V2 (Movement Only)", group = "AutoReal")
public class TestBlue extends OpMode {

    private Follower follower;
    private ElapsedTime stateTimer = new ElapsedTime();
    private int pathState = 0;

    // Poses
    private final Pose startPose = new Pose(20.968, 122.296, Math.toRadians(325));
    private final Pose shoot1 = new Pose(59.68609865470853, 84.11659192825114, Math.toRadians(310));
    private final Pose intake1 = new Pose(44.717488789237656, 84.30044843049328, Math.toRadians(180));

    // Paths
    private PathChain driveToShoot1, driveToIntake1;

    public void buildPaths() {
        driveToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot1.getHeading())
                .build();

        driveToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1, intake1))
                .setTangentHeadingInterpolation()
                .build();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start driving to Shoot position
                follower.followPath(driveToShoot1);
                setPathState(1);
                break;

            case 1: // Wait for arrival at Shoot, then drive to Intake
                // Added a small time buffer to ensure isBusy() has time to register
                if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
                    follower.followPath(driveToIntake1);
                    setPathState(2);
                }
                break;
        }
    }

    // Helper method to reset the timer every time the state changes
    private void setPathState(int state) {
        pathState = state;
        stateTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }
}