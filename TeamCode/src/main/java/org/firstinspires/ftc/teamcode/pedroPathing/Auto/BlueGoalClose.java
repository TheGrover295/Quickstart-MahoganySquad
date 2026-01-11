package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Blue Goal Auto Shoot - FIXED", group = "AutoReal")
public class BlueGoalClose extends OpMode {

    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime(); // Secondary timer for safety
    private int pathState = 0;
    private int totalShotsFired = 0;

    // Hardware
    private DcMotor flywheelMotor, chamberSpinner;
    private Servo artifactTransfer;

    // Poses
    private final Pose startPose = new Pose(20.968, 122.296, Math.toRadians(325));
    private final Pose shootPose = new Pose(60.364, 82.578, Math.toRadians(310));
    private final Pose parkPose = new Pose(44.498, 76.000, Math.toRadians(310));

    // Paths
    private PathChain driveToShoot, driveToPark;

    // Constants
    private final double TICKS_PER_STEP = 575.06;
    private final double SERVO_REST = 0.55;
    private final double SERVO_PUSH = 0.7;
    private double chamberTargetPos = 0;

    // Timing
    private final double INITIAL_WAIT = 1.0;
    private final double CHAMBER_WAIT = 3.0;
    private final double ATM_PUSH_TIME = 0.7;
    private final double TOTAL_SHOT_CYCLE = 2.0;

    public void buildPaths() {
        driveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveToPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");
        artifactTransfer = hardwareMap.get(Servo.class, "ATM");

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.63);

        artifactTransfer.setPosition(SERVO_REST);

        buildPaths();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start driving to Shoot
                follower.followPath(driveToShoot);
                setPathState(1);
                break;

            case 1: // Wait for arrival
                // Added a small time buffer to ensure isBusy() has time to register
                if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
                    setPathState(2);
                }
                break;

            case 2: // Short pause before shooting
                if (stateTimer.seconds() >= INITIAL_WAIT) {
                    setPathState(3);
                }
                break;

            case 3: // Rotate Chamber & Spool Flywheel
                moveChamberStep();
                flywheelMotor.setPower(0.67);
                setPathState(4);
                break;

            case 4: // Wait for Chamber to settle
                if (stateTimer.seconds() >= CHAMBER_WAIT) {
                    artifactTransfer.setDirection(Servo.Direction.REVERSE);
                    artifactTransfer.setPosition(SERVO_PUSH);
                    setPathState(5);
                }
                break;

            case 5: // Manage the Push and Reset
                // Retract servo after push time
                if (stateTimer.seconds() >= ATM_PUSH_TIME) {
                    artifactTransfer.setDirection(Servo.Direction.FORWARD);
                    artifactTransfer.setPosition(SERVO_REST);
                }

                // Check if the whole shot cycle is done
                if (stateTimer.seconds() >= TOTAL_SHOT_CYCLE) {
                    totalShotsFired++;
                    if (totalShotsFired < 3) {
                        setPathState(3); // Go again
                    } else {
                        flywheelMotor.setPower(0);
                        follower.followPath(driveToPark);
                        setPathState(6);
                    }
                }
                break;

            case 6: // Final parking check
                if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
                    setPathState(-1);
                }
                break;
        }
    }

    // Helper method to reset the timer every time the state changes
    private void setPathState(int state) {
        pathState = state;
        stateTimer.reset();
    }

    private void moveChamberStep() {
        chamberTargetPos += TICKS_PER_STEP;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setPower(0.6);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Shots", totalShotsFired);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Chamber Pos", chamberSpinner.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }
}