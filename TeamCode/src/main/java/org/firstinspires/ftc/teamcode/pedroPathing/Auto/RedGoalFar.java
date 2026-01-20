package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo; // Changed from Servo
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Red Goal Far", group = "AutoReal")
public class RedGoalFar extends OpMode {

    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime(); // Secondary timer for safety
    private int pathState = 0;
    private int totalShotsFired = 0;

    // Hardware
    private DcMotor flywheelMotor, chamberSpinner;
    private CRServo artifactTransfer; // Changed to CRServo to match BlueGoalClose

    // Poses
    private final Pose startPose = new Pose(58.58234977578478, 8.000035874439456, Math.toRadians(630));
    private final Pose shootPose = new Pose(71.95515695067265, 22.609865470852025, Math.toRadians(240));
    private final Pose intake1Pose = new Pose(15.457399103139023, 14.84304932735426);

    // Paths
    private PathChain driveToShoot, driveToIntake;

    // Constants
    private final double TICKS_PER_STEP = 490;
    private double chamberTargetPos = 0;
    private final double INITIAL_WAIT = 0.9;
    private final double CHAMBER_WAIT = 1.8; //1.8

    // --- UPDATED CONSTANTS ---
    // Increase this time specifically for the first stiff shot
    private final double ATM_PUSH_TIME_FIRST = 2.3;
    // This is the normal time for shots 2 and 3
    private final double ATM_PUSH_TIME_NORMAL = 0.9;
    // Increased to prevent the loop from cutting off the longer first shot
    private final double TOTAL_SHOT_CYCLE = 2.5;

    public void buildPaths() {
        driveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveToIntake = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake1Pose.getHeading())
                .build();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");

        // Updated hardware map for CRServo
        artifactTransfer = hardwareMap.get(CRServo.class, "ATM");

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.63);

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
                    flywheelMotor.setPower(0.76);
                }
                break;

            case 2: // Short pause before shooting
                if (stateTimer.seconds() >= INITIAL_WAIT) {
                    setPathState(3);
                }
                break;

            case 3: // Rotate Chamber & Spool Flywheel
                moveChamberStep();
                setPathState(4);
                break;

            case 4: // Wait for Chamber to settle
                if (stateTimer.seconds() >= CHAMBER_WAIT) {
                    // CRServo Logic: Set Power/Direction instead of Position
                    artifactTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
                    artifactTransfer.setPower(1);
                    setPathState(5);
                }
                break;

            case 5: // Manage the Push and Reset
                // LOGIC UPDATE: Pick the correct time based on which shot we are on
                double currentPushTime;

                if (totalShotsFired == 0) {
                    currentPushTime = ATM_PUSH_TIME_FIRST;
                } else {
                    currentPushTime = ATM_PUSH_TIME_NORMAL;
                }

                // Stop the servo when the selected time is reached
                if (stateTimer.seconds() >= currentPushTime) {
                    artifactTransfer.setPower(0);
                }

                // Check if the whole shot cycle is done
                if (stateTimer.seconds() >= TOTAL_SHOT_CYCLE) {
                    totalShotsFired++;
                    if (totalShotsFired < 3) {
                        setPathState(3); // Go again
                    } else {
                        flywheelMotor.setPower(0);
                        follower.followPath(driveToIntake);
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