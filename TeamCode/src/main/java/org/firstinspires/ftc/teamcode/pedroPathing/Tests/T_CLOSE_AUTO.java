package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.GoalTargeter;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.MotifDetector;

@Autonomous(name = "T_CLOSE_AUTO", group = "Tests")
public class T_CLOSE_AUTO extends LinearOpMode {

    // ===================== ALLIANCE SELECTION =====================
    private enum Alliance { RED, BLUE }
    private Alliance selectedAlliance = Alliance.BLUE;

    // ===================== SUBSYSTEMS =====================
    private Follower follower;
    private Limelight limelight;
    private GoalTargeter goalTargeter;
    private MotifDetector motifDetector;

    private DcMotorEx flywheelMotor;
    private DcMotor chamberSpinner;
    private DcMotor intakeMotor;
    private CRServo artifactTransfer;
    private Servo LimeServo;

    // ===================== TIMING =====================
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime shootTimer = new ElapsedTime();
    private final ElapsedTime intakeSeqTimer = new ElapsedTime();

    // ===================== STATE MACHINE =====================
    private enum AutoState {
        INIT,
        SCAN_MOTIF,
        SHOOT_PRELOADS,
        NAV_TO_PRE_INTAKE,
        INTAKE_DRIVE,
        PICKUP_BALLS,
        NAV_TO_SHOOT,
        ALIGN_AND_SHOOT,
        LEAVE_MARK,
        DONE
    }

    private AutoState currentState = AutoState.INIT;

    // ===================== CONFIGURATION =====================
    private static final double SCAN_TIMEOUT_SEC = 2.5;
    private static final double NAV_TIMEOUT_SEC = 5.0;
    private static final double PICKUP_TIMEOUT_SEC = 2.0;

    private static final double SHOOT_VELOCITY = 1025;

    private static final double CHAMBER_WAIT = 1.7;
    private static final double ATM_PUSH_TIME_FIRST = 0.9;
    private static final double ATM_PUSH_TIME_NORMAL = 0.9;

    // --- Chamber Stepper Variables ---
    private final double TICKS_PER_STEP = 475.06;
    private final double SHOOT_POS_TICKS = 100;
    private final double BACK_TO_INTAKE_TICKS = 100;
    private double chamberTargetPos = 0;

    // Intake Sequencing Variables
    private int intakeSeqStage = 0;
    private static final double INTAKE_SPIN_DELAY = 0.200;
    private static final double INTAKE_FIRST_DELAY = 0.700;

    // ===================== FIELD COORDINATES =====================

    private final Pose BLUE_START = new Pose(20.968, 122.296, Math.toRadians(325));
    private final Pose BLUE_SHOOT = new Pose(48, 96, Math.toRadians(318));

    private final Pose BLUE_INTAKE_GPP = new Pose(45, 36, Math.toRadians(-180)); //x =56
    private final Pose BLUE_INTAKE_PGP = new Pose(45, 60, Math.toRadians(-180));
    private final Pose BLUE_INTAKE_PPG = new Pose(45, 84, Math.toRadians(-180));

    private final Pose BLUE_INTAKE_GPP_END = new Pose(23, 36, Math.toRadians(-180)); // x= 25 and 35 for ppg
    private final Pose BLUE_INTAKE_PGP_END = new Pose(23, 60, Math.toRadians(-180));
    private final Pose BLUE_INTAKE_PPG_END = new Pose(33, 84, Math.toRadians(-180));

    // RED
    private final Pose RED_START = new Pose(122.672, 122.457, Math.toRadians(215));
    private final Pose RED_SHOOT = new Pose(96, 96, Math.toRadians(225));

    private final Pose RED_INTAKE_GPP = new Pose(103, 36, Math.toRadians(0)); // x= 90
    private final Pose RED_INTAKE_PGP = new Pose(103, 60, Math.toRadians(0)); // x= 88
    private final Pose RED_INTAKE_PPG = new Pose(103, 84, Math.toRadians(0)); // x= 88

    private final Pose RED_INTAKE_GPP_END = new Pose(122, 36, Math.toRadians(0)); // x= 132
    private final Pose RED_INTAKE_PGP_END = new Pose(122, 60, Math.toRadians(0)); // x= 132
    private final Pose RED_INTAKE_PPG_END = new Pose(117, 84, Math.toRadians(0)); // x= 127
    private final Pose LEAVE_MARK_RED = new Pose(97, 73, Math.toRadians(225));
    private final Pose LEAVE_MARK_BLUE = new Pose(48.569, 71.869, Math.toRadians(318));

    // Active Points
    private Pose startPose;
    private Pose shootPose;
    private Pose preIntakePose;
    private Pose finalIntakePose;

    private PathChain currentPath;

    // --- VARIABLES ---
    private MotifDetector.Motif detectedMotif = MotifDetector.Motif.UNKNOWN;
    private int ballsShot = 0;
    private int shootSubState = 0;
    private boolean inSecondShootingPhase = false;
    private String decisionReason = "Waiting";

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        // ===================== SELECTION LOOP =====================
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.left_bumper) selectedAlliance = Alliance.RED;
            else if (gamepad1.right_bumper) selectedAlliance = Alliance.BLUE;

            goalTargeter.update();
            motifDetector.update(goalTargeter.getVisionData());

            // Camera servo for better motif visibility during selection
            if (selectedAlliance == Alliance.BLUE) LimeServo.setPosition(0.75);
            else LimeServo.setPosition(0.25);

            telemetry.addLine("=== LIMELIGHT CLOSE (ALLIANCE SELECTION) ===");
            telemetry.addData("Selected Alliance", selectedAlliance);
            telemetry.addLine("LB = RED | RB = BLUE");
            telemetry.addLine();
            telemetry.addData("Motif (Live)", motifDetector.getDetectedMotif());
            telemetry.update();
        }

        // ===================== SETUP BASED ON SELECTION =====================
        if (selectedAlliance == Alliance.BLUE) {
            startPose = BLUE_START;
            shootPose = BLUE_SHOOT;
        } else {
            startPose = RED_START;
            shootPose = RED_SHOOT;
        }

        follower.setStartingPose(startPose);

        // FIX: build from follower pose estimate (after setting starting pose) to shoot pose
        followTo(shootPose);

        flywheelMotor.setVelocity(SHOOT_VELOCITY);

        runtime.reset();
        currentState = AutoState.SCAN_MOTIF;
        stateTimer.reset();

        // ===================== RUN LOOP =====================
        while (opModeIsActive()) {
            follower.update();
            goalTargeter.update();
            motifDetector.update(goalTargeter.getVisionData());

            switch (currentState) {
                case SCAN_MOTIF: runScanMotif(); break;
                case SHOOT_PRELOADS: runShootPreloads(); break;
                case NAV_TO_PRE_INTAKE: runNavToPreIntake(); break;
                case INTAKE_DRIVE: runIntakeDrive(); break;
                case PICKUP_BALLS: runPickupBalls(); break;
                case NAV_TO_SHOOT: runNavToShoot(); break;
                case ALIGN_AND_SHOOT: runAlignAndShoot(); break;
                case LEAVE_MARK: runLeaveMark(); break;
                case DONE:
                    stopAllMechanisms();
                    follower.breakFollowing();
                    break;
                default:
                    break;
            }
            updateTelemetry();
        }
    }

    private void initHardware() {
        follower = Constants.createFollower(hardwareMap);

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        artifactTransfer = hardwareMap.get(CRServo.class, "ATM");
        LimeServo = hardwareMap.get(Servo.class, "axonLime");

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients pidfNew = new PIDFCoefficients(20.3025, 0, 0, 20.7020);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.63);

        limelight = new Limelight();
        limelight.init(hardwareMap);
        limelight.switchPipeline(0);
        goalTargeter = new GoalTargeter(limelight);
        motifDetector = new MotifDetector();

        LimeServo.setPosition(0.5);
    }

    // ===================== LOGIC =====================

    private void runScanMotif() {
        if (detectedMotif == MotifDetector.Motif.UNKNOWN && motifDetector.hasConfidentDetection()) {
            detectedMotif = motifDetector.getDetectedMotif();
            decisionReason = "Confident (Drive)";
            RobotLog.d("AUTO", "Motif Locked (Confident): " + detectedMotif);
        }

        if (!follower.isBusy()) {
            if (detectedMotif == MotifDetector.Motif.UNKNOWN) {
                MotifDetector.Motif lastSeen = motifDetector.getDetectedMotif();
                if (lastSeen != MotifDetector.Motif.UNKNOWN) {
                    detectedMotif = lastSeen;
                    decisionReason = "Arrived (Weak)";
                } else {
                    detectedMotif = MotifDetector.Motif.GPP;
                    decisionReason = "Arrived (Default)";
                }
                RobotLog.d("AUTO", "Motif Locked (Final): " + detectedMotif);
            }

            LimeServo.setPosition(0.5);
            transitionTo(AutoState.SHOOT_PRELOADS);
        } else if (stateTimer.seconds() > SCAN_TIMEOUT_SEC) {
            // Optional: move on if something stalls
            transitionTo(AutoState.SHOOT_PRELOADS);
        }
    }

    private void runShootPreloads() {
        if (follower.isBusy()) return;
        runShootingLogic(false);
    }

    private void runNavToPreIntake() {
        if (!follower.isBusy() && stateTimer.seconds() > 0.3) {
            intakeMotor.setPower(1.0);

            chamberTargetPos -= BACK_TO_INTAKE_TICKS;
            chamberSpinner.setTargetPosition((int) chamberTargetPos);
            chamberSpinner.setPower(1);

            // FIX: reset sequencing timers when you start intake sequence
            intakeSeqStage = 0;
            intakeSeqTimer.reset();

            // FIX: do NOT start a path at preIntakePose constant—start from current pose estimate
            followTo(finalIntakePose);

            transitionTo(AutoState.INTAKE_DRIVE);
        } else if (stateTimer.seconds() > NAV_TIMEOUT_SEC) {
            intakeMotor.setPower(1.0);

            chamberTargetPos -= BACK_TO_INTAKE_TICKS;
            chamberSpinner.setTargetPosition((int) chamberTargetPos);
            chamberSpinner.setPower(1);

            intakeSeqStage = 0;
            intakeSeqTimer.reset();

            followTo(finalIntakePose);

            transitionTo(AutoState.INTAKE_DRIVE);
        }
    }

    private void runIntakeDrive() {
        updateIntakeIndexing();

        if (!follower.isBusy() && stateTimer.seconds() > 0.3) {
            transitionTo(AutoState.PICKUP_BALLS);
        } else if (stateTimer.seconds() > 3.0) {
            transitionTo(AutoState.PICKUP_BALLS);
        }
    }

    private void runPickupBalls() {
        updateIntakeIndexing();

        if (stateTimer.seconds() > PICKUP_TIMEOUT_SEC) {
            // FIX: drive back to shoot from current pose estimate, not from finalIntakePose constant
            followTo(shootPose);

            flywheelMotor.setVelocity(SHOOT_VELOCITY);
            transitionTo(AutoState.NAV_TO_SHOOT);
        }
    }

    private void updateIntakeIndexing() {
        switch (intakeSeqStage) {
            case 0:
                if (intakeSeqTimer.seconds() >= INTAKE_FIRST_DELAY) {
                    moveChamberStep();
                    intakeSeqTimer.reset();
                    intakeSeqStage = 1;
                }
                break;
            case 1:
                if (intakeSeqTimer.seconds() >= INTAKE_SPIN_DELAY) {
                    moveChamberStep();
                    intakeSeqTimer.reset();
                    intakeSeqStage = 2;
                }
                break;
            case 2:
                if (intakeSeqTimer.seconds() >= INTAKE_SPIN_DELAY) {
                    moveChamberStep();
                    intakeSeqTimer.reset();
                    intakeSeqStage = 3;
                }
                break;
            case 3:
                if (intakeSeqTimer.seconds() >= INTAKE_SPIN_DELAY) {
                    moveChamberStep();
                    intakeSeqTimer.reset();
                    intakeSeqStage = 4;
                }
                break;
            case 4:
            default:
                break;
        }
    }

    private void runNavToShoot() {
        if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
            intakeMotor.setPower(0);
            startSecondShootingPhase();
        } else if (stateTimer.seconds() > NAV_TIMEOUT_SEC) {
            startSecondShootingPhase();
        }
    }

    private void startSecondShootingPhase() {
        chamberTargetPos += SHOOT_POS_TICKS;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setPower(1);

        inSecondShootingPhase = true;
        ballsShot = 0;
        shootSubState = 0;
        transitionTo(AutoState.ALIGN_AND_SHOOT);
    }

    private void runAlignAndShoot() {
        flywheelMotor.setVelocity(SHOOT_VELOCITY);
        runShootingLogic(true);
    }

    private void runShootingLogic(boolean isSecondPhase) {
        switch (shootSubState) {
            case 0:
                if (ballsShot > 0) moveChamberStep();
                shootTimer.reset();
                shootSubState = 1;
                break;

            case 1:
                if (shootTimer.seconds() >= CHAMBER_WAIT) {
                    artifactTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
                    artifactTransfer.setPower(1);
                    shootTimer.reset();
                    shootSubState = 2;
                }
                break;

            case 2:
                double pushTime = (ballsShot == 0 && !isSecondPhase) ? ATM_PUSH_TIME_FIRST : ATM_PUSH_TIME_NORMAL;
                if (shootTimer.seconds() >= pushTime) {
                    artifactTransfer.setPower(0);
                    shootSubState = 3;
                }
                break;

            case 3:
                ballsShot++;
                if (ballsShot >= 3) {
                    flywheelMotor.setVelocity(0);

                    if (!isSecondPhase) {
                        setTargetForMotif();
                        transitionTo(AutoState.NAV_TO_PRE_INTAKE);
                    } else {
                        Pose leavePose = (selectedAlliance == Alliance.BLUE) ? LEAVE_MARK_BLUE : LEAVE_MARK_RED;

                        // FIX: build leave path from current pose estimate, not from shootPose constant
                        followTo(leavePose);

                        transitionTo(AutoState.LEAVE_MARK);
                    }
                } else {
                    shootSubState = 0;
                }
                break;

            default:
                shootSubState = 0;
                break;
        }
    }

    private void runLeaveMark() {
        if (!follower.isBusy()) {
            transitionTo(AutoState.DONE);
        } else if (stateTimer.seconds() > NAV_TIMEOUT_SEC) {
            transitionTo(AutoState.DONE);
        }
    }

    private void setTargetForMotif() {
        if (selectedAlliance == Alliance.BLUE) {
            switch (detectedMotif) {
                case GPP: preIntakePose = BLUE_INTAKE_GPP; finalIntakePose = BLUE_INTAKE_GPP_END; break;
                case PGP: preIntakePose = BLUE_INTAKE_PGP; finalIntakePose = BLUE_INTAKE_PGP_END; break;
                case PPG: preIntakePose = BLUE_INTAKE_PPG; finalIntakePose = BLUE_INTAKE_PPG_END; break;
                default: preIntakePose = BLUE_INTAKE_GPP; finalIntakePose = BLUE_INTAKE_GPP_END; break;
            }
        } else {
            switch (detectedMotif) {
                case GPP: preIntakePose = RED_INTAKE_GPP; finalIntakePose = RED_INTAKE_GPP_END; break;
                case PGP: preIntakePose = RED_INTAKE_PGP; finalIntakePose = RED_INTAKE_PGP_END; break;
                case PPG: preIntakePose = RED_INTAKE_PPG; finalIntakePose = RED_INTAKE_PPG_END; break;
                default: preIntakePose = RED_INTAKE_GPP; finalIntakePose = RED_INTAKE_GPP_END; break;
            }
        }

        RobotLog.d("AUTO", "Targets: Pre=" + preIntakePose + " | Final=" + finalIntakePose);

        // FIX: drive to preIntake from current pose estimate
        followTo(preIntakePose);
    }

    /**
     * FIX: Always build from the robot's CURRENT estimated pose.
     * This removes discontinuities and the “17 inch” style systematic offsets you’ve been seeing.
     */
    private void followTo(Pose target) {
        Pose start = follower.getPose();
        currentPath = follower.pathBuilder()
                .addPath(new BezierLine(start, target))
                .setLinearHeadingInterpolation(start.getHeading(), target.getHeading())
                .build();
        follower.followPath(currentPath);
    }

    // Kept for compatibility if other code references it, but now delegates to followTo.
    private void buildAndFollowPath(Pose start, Pose end) {
        // FIX: ignore passed-in start and use follower pose estimate for correctness
        followTo(end);
    }

    private void transitionTo(AutoState newState) {
        currentState = newState;
        stateTimer.reset();
    }

    private void stopAllMechanisms() {
        flywheelMotor.setVelocity(0);
        intakeMotor.setPower(0);
        artifactTransfer.setPower(0);
    }

    private void moveChamberStep() {
        chamberTargetPos += TICKS_PER_STEP;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setPower(1);
    }

    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Motif", detectedMotif);
        telemetry.addData("Decision", decisionReason);
        telemetry.addData("Flywheel Vel", flywheelMotor.getVelocity());
        telemetry.addData("Pose", follower.getPose());
        if (finalIntakePose != null) {
            telemetry.addData("Final Target", "X=%.1f Y=%.1f",
                    finalIntakePose.getX(), finalIntakePose.getY());
        }
        telemetry.update();
    }
}