package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Blue Goal Far V2", group = "AutoReal")
public class BlueFarV2 extends OpMode {

    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();
    private int pathState = 0;

    // Logic Variables
    private int totalShotsFired = 0;
    private int cycleCount = 0; // Tracks which major cycle we are on (0, 1, or 2)

    // Hardware
    private DcMotor flywheelMotor, chamberSpinner;
    private CRServo artifactTransfer;

    // Poses
    private final Pose startPose = new Pose(62.13391928251123, 7.031426008968618, Math.toRadians(630));

    // Cycle 1 Poses
    private final Pose shootPose = new Pose(62.591928251121075, 18.896860986547097, Math.toRadians(295));
    private final Pose intake1Pose = new Pose(129.26905829596413, 12.09865470852018);

    // Cycle 2 Poses
    private final Pose shoot2Pose = new Pose(62.591928251121075, 18.896860986547097, Math.toRadians(295));
    private final Pose intake2Pose = new Pose(129.26905829596413, 12.09865470852018);

    // Cycle 3 Poses
    private final Pose shoot3Pose = new Pose(62.591928251121075, 18.896860986547097, Math.toRadians(295));
    private final Pose intake3Pose = new Pose(129.26905829596413, 12.09865470852018);

    // Paths
    private PathChain driveToShoot1, driveToIntake1;
    private PathChain driveToShoot2, driveToIntake2;
    private PathChain driveToShoot3, driveToIntake3;

    // Constants
    private final double TICKS_PER_STEP = 480;
    private double chamberTargetPos = 0;
    private final double INITIAL_WAIT = 1.0;
    private final double CHAMBER_WAIT = 1.9;

    // --- SHOOTING CONSTANTS ---
    private final double ATM_PUSH_TIME_FIRST = 2.3;
    private final double ATM_PUSH_TIME_NORMAL = 0.9; //0.9
    private final double TOTAL_SHOT_CYCLE = 2.5;

    public void buildPaths() {
        // Cycle 1
        driveToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake1Pose.getHeading())
                .build();

        // Cycle 2 (Connects from Intake 1 -> Shoot 2 -> Intake 2)
        driveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, shoot2Pose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), shoot2Pose.getHeading())
                .build();

        driveToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, intake2Pose))
                .setLinearHeadingInterpolation(shoot2Pose.getHeading(), intake2Pose.getHeading())
                .build();

        // Cycle 3 (Connects from Intake 2 -> Shoot 3 -> Intake 3)
        driveToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shoot3Pose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shoot3Pose.getHeading())
                .build();

        driveToIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot3Pose, intake3Pose))
                .setLinearHeadingInterpolation(shoot3Pose.getHeading(), intake3Pose.getHeading())
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

        buildPaths();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start driving to Initial Shoot Position
                follower.followPath(driveToShoot1);
                setPathState(1);
                break;

            case 1: // Wait for arrival at ANY Shoot Position
                // This state is reused for Shoot 1, Shoot 2, and Shoot 3 arrival
                if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
                    setPathState(2);
                    flywheelMotor.setPower(0.8);
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
                    artifactTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
                    artifactTransfer.setPower(1);
                    setPathState(5);
                }
                break;

            case 5: // Shooting Logic
                double currentPushTime;

                // Use longer push time for the very first shot of the match only
                if (totalShotsFired == 0) {
                    currentPushTime = ATM_PUSH_TIME_FIRST;
                } else {
                    currentPushTime = ATM_PUSH_TIME_NORMAL;
                }

                if (stateTimer.seconds() >= currentPushTime) {
                    artifactTransfer.setPower(0);
                }

                if (stateTimer.seconds() >= TOTAL_SHOT_CYCLE) {
                    totalShotsFired++;
                    if (totalShotsFired < 3) {
                        setPathState(3); // Shoot again
                    } else {
                        // Finished 3 shots. Stop flywheel.
                        flywheelMotor.setPower(0);

                        // Decide which Intake to go to based on current Cycle
                        if (cycleCount == 0) {
                            follower.followPath(driveToIntake1);
                        } else if (cycleCount == 1) {
                            follower.followPath(driveToIntake2);
                        } else if (cycleCount == 2) {
                            follower.followPath(driveToIntake3);
                        }

                        setPathState(6); // Move to Intake Logic
                    }
                }
                break;

            case 6: // Wait for arrival at ANY Intake Position
                if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
                    setPathState(7);
                }
                break;

            // --- INTAKE LOGIC (Loops 3 times) ---

            case 7: // 1st Intake Step: Wait 2s -> Spin
                if (stateTimer.seconds() >= 2.0) {
                    moveChamberStep();
                    setPathState(8);
                }
                break;

            case 8: // 2nd Intake Step: Wait 2s -> Spin
                if (stateTimer.seconds() >= 2.0) {
                    moveChamberStep();
                    setPathState(9);
                }
                break;

            case 9: // 3rd Intake Step & Cycle Check
                if (stateTimer.seconds() >= 2.0) {
                    //moveChamberStep();

                    // Intake finished. Check if we need to do another Cycle.
                    cycleCount++; // Increment cycle (0 -> 1, or 1 -> 2)

                    if (cycleCount == 1) {
                        // Done with Cycle 1, Start Cycle 2
                        totalShotsFired = 0; // Reset shots for next round
                        follower.followPath(driveToShoot2);
                        setPathState(1); // Jump back to Wait for Arrival

                    } else if (cycleCount == 2) {
                        // Done with Cycle 2, Start Cycle 3
                        totalShotsFired = 0; // Reset shots for next round
                        follower.followPath(driveToShoot3);
                        setPathState(1); // Jump back to Wait for Arrival

                    } else {
                        // Done with Cycle 3 (All done)
                        setPathState(-1);
                    }
                }
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

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Cycle", cycleCount);
        telemetry.addData("Shots Fired", totalShotsFired);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }
}