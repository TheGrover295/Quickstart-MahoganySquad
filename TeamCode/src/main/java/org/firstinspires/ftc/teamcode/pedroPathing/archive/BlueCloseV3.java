package org.firstinspires.ftc.teamcode.pedroPathing.archive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime; // Import Timer
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "BlueCloseV3", group = "archive")
@Configurable
public class BlueCloseV3 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private ElapsedTime pathTimer; // Timer for our waits

    // --- State Machine Constants ---
    private int pathState;
    private static final int START_STATE = 0;
    private static final int PATH_5_STATE = 1;       // Driving to shoot position
    private static final int WAIT_SHOOT_STATE = 2;   // Waiting for shot to fire
    private static final int PATH_3_STATE = 3;       // Driving to next position
    private static final int END_STATE = 4;

    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Timer
        pathTimer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.968, 122.296, Math.toRadians(325)));

        paths = new Paths(follower);
        pathState = START_STATE;

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
        panelsTelemetry.update(telemetry);
    }

    // --- State Machine with Wait Logic ---
    public int autonomousPathUpdate() {
        switch (pathState) {
            case START_STATE:
                // Start moving on Path 5 immediately
                follower.followPath(paths.Path5);
                return PATH_5_STATE;

            case PATH_5_STATE:
                // Check if we reached the shooting position
                if (!follower.isBusy()) {
                    // Reset the timer before entering the wait state
                    pathTimer.reset();
                    return WAIT_SHOOT_STATE;
                }
                break;

            case WAIT_SHOOT_STATE:
                // Logic: Trigger your shooter mechanism here (e.g., servo.setPosition)

                // Wait for 1000 milliseconds (1 second)
                if (pathTimer.milliseconds() > 3000) {
                    // After wait is done, start Path 3
                    follower.followPath(paths.Path3);
                    return PATH_3_STATE;
                }
                break;

            case PATH_3_STATE:
                // Check if Path 3 is done
                if (!follower.isBusy()) {
                    return END_STATE;
                }
                break;

            case END_STATE:
                // Stop or hold position
                break;
        }
        // No state change, return current state
        return pathState;
    }

    public static class Paths {
        public PathChain Path5;
        public PathChain Path3;

        public Paths(Follower follower) {
            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(20.968, 122.296),
                            new Pose(59.686, 84.117)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(318))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(59.686, 84.117),
                            new Pose(66.601, 58.233),
                            new Pose(60.473, 66.000) //66.000 NEW X = 37.73
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(180))
                    .build();

        }
    }
}