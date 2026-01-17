package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime; // Added for the timer
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class BlueGoalCloseV3 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private ElapsedTime opmodeTimer; // Timer object

    // Starting Pose (Adjust if needed)
    private final Pose startPose = new Pose(20.968, 122.296, Math.toRadians(325));

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        opmodeTimer = new ElapsedTime(); // Initialize timer

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        paths = new Paths(follower, startPose);
        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Timer", opmodeTimer.seconds());
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Trigger Path: Start -> Shoot 1
                follower.followPath(paths.shoot1, true);
                return 1; // Move to next state immediately to wait for finish

            case 1: // Wait for Robot to Arrive at Shoot 1
                if(!follower.isBusy()) {
                    // Robot has arrived. Reset the timer for the wait.
                    opmodeTimer.reset();
                    return 2;
                }
                break;

            case 2: // WAIT 5 SECONDS
                if(opmodeTimer.seconds() >= 5) {
                    // Time is up! Trigger the Turn.
                    follower.followPath(paths.turn1, true);
                    return 3;
                }
                break;

            case 3: // Wait for Turn to Finish
                if(!follower.isBusy()) {
                    // Turn is done. Trigger Drive to Intake.
                    follower.followPath(paths.intake1, true);
                    return 4;
                }
                break;

            case 4: // Wait for Intake Path to Finish
                if(!follower.isBusy()) {
                    // Robot arrived at Intake. Trigger Backup.
                    follower.followPath(paths.backup1, true);
                    return 5;
                }
                break;

            case 5: // Wait for Backup to Finish
                if(!follower.isBusy()) {
                    // Backup done. Trigger Shoot 2.
                    follower.followPath(paths.shoot2, true);
                    return 6;
                }
                break;

            case 6: // Wait for Shoot 2 to Finish
                if(!follower.isBusy()) {
                    return 7; // End
                }
                break;

            case 7: // End of Auto
                break;
        }
        return pathState; // Return current state if no change
    }

    public static class Paths {
        public PathChain shoot1;
        public PathChain turn1; // New Turn Path
        public PathChain intake1;
        public PathChain backup1;
        public PathChain shoot2;

        public Paths(Follower follower, Pose startPose) {
            // 1. Drive to Shoot Position
            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            startPose,
                            new Pose(59.557, 83.869)
                    ))
                    .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(310))
                    .build();

            // 2. Turn In Place (Same Start & End Coordinate, Changing Heading)
            turn1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(59.557, 83.869),
                            new Pose(59.557, 83.869)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(310), Math.toRadians(180))
                    .build();

            // 3. Drive to Intake (Straight line, Constant Heading)
            intake1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(59.557, 83.869),
                            new Pose(15.117, 84.283)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // 4. Backup
            backup1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(15.117, 84.283),
                            new Pose(32.188, 84.175)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // 5. Shoot 2
            shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(32.188, 84.175),
                            new Pose(59.557, 83.869)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(310))
                    .build();
        }
    }
}