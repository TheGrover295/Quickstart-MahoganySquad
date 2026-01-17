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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Blue Goal Close V2 ", group = "AutoReal")
public class BlueGoalGloseV2 extends OpMode {

    private Follower follower;
    private ElapsedTime stateTimer = new ElapsedTime();
    private int pathState = 0;
    private int totalShotsFired = 0;

    // Hardware
    private DcMotor flywheelMotor, chamberSpinner;
    private CRServo artifactTransfer;

    // Poses
    private final Pose startPose = new Pose(20.968, 122.296, Math.toRadians(325));
    private final Pose shoot1 = new Pose(59.557, 83.869, Math.toRadians(310));
    private final Pose preintake1 = new Pose(42.61883408071749, 83.94618834080718, Math.toRadians(180));
    private final Pose intake1 = new Pose(23.99592376681614, 84.12156502242154, Math.toRadians(180)); //180
    private final Pose shoot2 = new Pose(59.557, 83.869, Math.toRadians(310));

    // Paths
    private PathChain driveToShoot1, driveToIntake1, driveToBackup1, driveToShoot2;

    // Constants
    private final double TICKS_PER_STEP = 575.06;
    private final double TICKS_SMALL_STEP = 100.0;

    private double chamberTargetPos = 0;

    // Timing
    private final double INITIAL_WAIT = 0.5;
    private final double CHAMBER_WAIT = 3.0; // Time allowed for chamber to rotate before feeding
    private final double FEEDER_SPIN_TIME = 0.5; // How long the servo spins to push ring in
    private final double TOTAL_SHOT_CYCLE = 1.5; // Total time per shot (must be > FEEDER_SPIN_TIME)

    public void buildPaths() {
        driveToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot1.getHeading())
                .build();

        driveToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1, preintake1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        driveToBackup1 = follower.pathBuilder()
                .addPath(new BezierLine(preintake1, intake1))
                .setLinearHeadingInterpolation(preintake1.getHeading(), intake1.getHeading())
                .build();

        driveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, shoot2))
                .setLinearHeadingInterpolation(intake1.getHeading(), shoot2.getHeading())
                .build();


    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");
        artifactTransfer = hardwareMap.get(CRServo.class, "ATM");

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.63);

        // Make sure feeder is stopped
        artifactTransfer.setPower(0);

        buildPaths();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start driving to Shoot 1
                follower.followPath(driveToShoot1);
                setPathState(1);
                break;

            case 1: // Wait for arrival at Shoot 1
                if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
                    setPathState(2);
                    flywheelMotor.setPower(0.67);
                }
                break;

            case 2: // Short pause before shooting
                if (stateTimer.seconds() >= INITIAL_WAIT) {
                    setPathState(3);
                }
                break;

            case 3: // Rotate Chamber & Spool Flywheel
                moveChamberStep();
                moveChamberSmallStep();
                setPathState(4);
                break;

            case 4: // Wait for Chamber to align, then start feeding
                if (stateTimer.seconds() >= CHAMBER_WAIT) {

                    artifactTransfer.setPower(1.0);
                    setPathState(5);
                }
                break;

            case 5: // Feed ring, Stop, Reset

                if (stateTimer.seconds() >= FEEDER_SPIN_TIME) {
                    artifactTransfer.setPower(0);
                }


                if (stateTimer.seconds() >= TOTAL_SHOT_CYCLE) {
                    // Double check it's stopped
                    artifactTransfer.setPower(0);

                    totalShotsFired++;
                    if (totalShotsFired < 3) {
                        setPathState(3); // Loop back for next shot
                    } else {
                        flywheelMotor.setPower(0);
                        follower.followPath(driveToIntake1);
                        setPathState(6);
                    }
                }
                break;

            case 6: // Arriving at Intake
                if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
                    follower.followPath(driveToBackup1);
                    setPathState(7);
                }
                break;

            case 7: // Arriving at Backup
                if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
                    follower.followPath(driveToShoot2);
                    flywheelMotor.setPower(0.67); // Spin up early
                    setPathState(8);
                }
                break;

            case 8: // Driving to Shoot 2
                if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
                    setPathState(9);
                }
                break;

            case 9: // Settling at Shoot 2
                if (stateTimer.seconds() >= INITIAL_WAIT) {
                    totalShotsFired = 0; // Reset counter
                    setPathState(10);
                }
                break;

            case 10: // Restart Shooting Cycle
                setPathState(3); // Jump back to shooting logic
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
        stateTimer.reset();
    }

    private void moveChamberStep() {
        chamberTargetPos += TICKS_PER_STEP;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setPower(0.6);
    }

    private void moveChamberSmallStep() {
        chamberTargetPos += TICKS_SMALL_STEP;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setPower(0.6);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Shots", totalShotsFired);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }
}