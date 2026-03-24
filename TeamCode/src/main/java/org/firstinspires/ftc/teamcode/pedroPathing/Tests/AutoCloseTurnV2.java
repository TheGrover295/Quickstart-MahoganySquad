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

@Autonomous(name = "AutoCloseTurn V2", group = "Tests")
public class AutoCloseTurnV2 extends LinearOpMode {

    // ===================== ALLIANCE =====================
    private enum Alliance {
        RED,
        BLUE
    }

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
    private Servo limeServo;

    // ===================== TIMERS =====================
    private final ElapsedTime runtime        = new ElapsedTime();
    private final ElapsedTime stateTimer     = new ElapsedTime();
    private final ElapsedTime shootTimer     = new ElapsedTime();
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

    // ===================== CONFIG =====================
    private static final double SCAN_TIMEOUT_SEC   = 2.5;
    private static final double NAV_TIMEOUT_SEC    = 5.0;
    private static final double PICKUP_TIMEOUT_SEC = 2.0;

    private static final double SHOOT_VELOCITY = 1035;

    private static final double CHAMBER_WAIT          = 1.7;
    private static final double ATM_PUSH_TIME_FIRST   = 0.9;
    private static final double ATM_PUSH_TIME_NORMAL  = 0.9;

    // Small positional nudge (inches) so Pedro never gets a zero-length segment
    // when the robot turns in place. Tune this to 0 if your Pedro build handles
    // zero-distance segments correctly; otherwise keep at 1.0–2.0 inches.
    private static final double TURN_NUDGE_INCHES = 1.5;

    private final double TICKS_PER_STEP       = 475.06;
    private final double SHOOT_POS_TICKS      = 100;
    private final double BACK_TO_INTAKE_TICKS = 100;
    private double chamberTargetPos = 0;

    private int intakeSeqStage = 0;
    private static final double INTAKE_SPIN_DELAY  = 0.200;
    private static final double INTAKE_FIRST_DELAY = 3.0;

    // ===================== FIELD COORDINATES =====================

    // --- BLUE ---
    private final Pose BLUE_START = new Pose(20.968, 122.296, Math.toRadians(325));
    private final Pose BLUE_SHOOT = new Pose(52.686, 96.116, Math.toRadians(318));

    private final Pose BLUE_INTAKE_GPP = new Pose(45, 36,  Math.toRadians(180));
    private final Pose BLUE_INTAKE_PGP = new Pose(45, 60,  Math.toRadians(180));
    private final Pose BLUE_INTAKE_PPG = new Pose(45, 84,  Math.toRadians(180));

    private final Pose BLUE_INTAKE_GPP_END = new Pose(37, 36,  Math.toRadians(180));
    private final Pose BLUE_INTAKE_PGP_END = new Pose(37, 60,  Math.toRadians(180));
    private final Pose BLUE_INTAKE_PPG_END = new Pose(42, 94,  Math.toRadians(180));

    private final Pose LEAVE_MARK_BLUE = new Pose(48, 71, Math.toRadians(318));

    // --- RED ---
    private final Pose RED_START = new Pose(122.672, 122.457, Math.toRadians(215));
    private final Pose RED_SHOOT = new Pose(88.314,   91.116, Math.toRadians(225));

    private final Pose RED_INTAKE_GPP = new Pose(102, 36,  Math.toRadians(0));
    private final Pose RED_INTAKE_PGP = new Pose(102, 48,  Math.toRadians(0));
    private final Pose RED_INTAKE_PPG = new Pose(102, 84,  Math.toRadians(0));

    private final Pose RED_INTAKE_GPP_END = new Pose(102, 36,  Math.toRadians(0));
    private final Pose RED_INTAKE_PGP_END = new Pose(112, 60,  Math.toRadians(0));
    private final Pose RED_INTAKE_PPG_END = new Pose(115, 84,  Math.toRadians(0));

    private final Pose LEAVE_MARK_RED = new Pose(97, 73, Math.toRadians(225));

    // ===================== ACTIVE TARGETS =====================
    private Pose startPose;
    private Pose shootPose;
    private Pose preIntakePose;
    private Pose finalIntakePose;

    private PathChain currentPath;

    // ===================== RUNTIME VARIABLES =====================
    private MotifDetector.Motif detectedMotif = MotifDetector.Motif.UNKNOWN;
    private int     ballsShot             = 0;
    private int     shootSubState         = 0;
    private boolean inSecondShootingPhase = false;
    private String  decisionReason        = "Waiting";

    // ===================== MAIN =====================

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        // ---- Pre-match loop: alliance selection + live motif preview ----
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.left_bumper) {
                selectedAlliance = Alliance.RED;
            } else if (gamepad1.right_bumper) {
                selectedAlliance = Alliance.BLUE;
            }

            goalTargeter.update();
            motifDetector.update(goalTargeter.getVisionData());

            telemetry.addLine("=== LIMELIGHT CLOSE (ALLIANCE SELECTION) ===");
            telemetry.addData("Selected Alliance", selectedAlliance);
            telemetry.addLine("LB = RED | RB = BLUE");
            telemetry.addLine();
            telemetry.addData("Motif (Live)", motifDetector.getDetectedMotif());
            telemetry.update();
        }

        // ---- Set alliance-specific starting data ----
        if (selectedAlliance == Alliance.BLUE) {
            startPose = BLUE_START;
            shootPose = BLUE_SHOOT;
            limeServo.setPosition(0.75);
        } else {
            startPose = RED_START;
            shootPose = RED_SHOOT;
            limeServo.setPosition(0.25);
        }

        follower.setStartingPose(startPose);

        // Initial drive to shoot pose
        followLinearHeadingPath(startPose, shootPose);

        flywheelMotor.setVelocity(SHOOT_VELOCITY);
        runtime.reset();
        currentState = AutoState.SCAN_MOTIF;
        stateTimer.reset();

        // ---- Main loop ----
        while (opModeIsActive()) {
            follower.update();
            goalTargeter.update();
            motifDetector.update(goalTargeter.getVisionData());

            switch (currentState) {
                case SCAN_MOTIF:      runScanMotif();      break;
                case SHOOT_PRELOADS:  runShootPreloads();  break;
                case NAV_TO_PRE_INTAKE: runNavToPreIntake(); break;
                case INTAKE_DRIVE:    runIntakeDrive();    break;
                case PICKUP_BALLS:    runPickupBalls();    break;
                case NAV_TO_SHOOT:    runNavToShoot();     break;
                case ALIGN_AND_SHOOT: runAlignAndShoot();  break;
                case LEAVE_MARK:      runLeaveMark();      break;
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

    // ===================== HARDWARE INIT =====================

    private void initHardware() {
        follower = Constants.createFollower(hardwareMap);

        flywheelMotor  = hardwareMap.get(DcMotorEx.class, "flywheel");
        chamberSpinner = hardwareMap.get(DcMotor.class,   "chamberSpinner");
        intakeMotor    = hardwareMap.get(DcMotor.class,   "intakeMotor");
        artifactTransfer = hardwareMap.get(CRServo.class, "ATM");
        limeServo      = hardwareMap.get(Servo.class,     "axonLime");

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

        goalTargeter  = new GoalTargeter(limelight);
        motifDetector = new MotifDetector();

        limeServo.setPosition(0.5);
    }

    // ===================== STATE LOGIC =====================

    private void runScanMotif() {
        // Try to lock motif confidently while driving
        if (detectedMotif == MotifDetector.Motif.UNKNOWN
                && motifDetector.hasConfidentDetection()) {
            detectedMotif = motifDetector.getDetectedMotif();
            decisionReason = "Confident (Drive)";
            RobotLog.d("AUTO", "Motif Locked (Confident): " + detectedMotif);
        }

        boolean arrived = !follower.isBusy();
        boolean timedOut = stateTimer.seconds() > SCAN_TIMEOUT_SEC;

        if (arrived || timedOut) {
            if (detectedMotif == MotifDetector.Motif.UNKNOWN) {
                MotifDetector.Motif lastSeen = motifDetector.getDetectedMotif();
                if (lastSeen != MotifDetector.Motif.UNKNOWN) {
                    detectedMotif = lastSeen;
                    decisionReason = arrived ? "Arrived (Weak)" : "Timeout (Weak)";
                } else {
                    detectedMotif = MotifDetector.Motif.GPP;
                    decisionReason = arrived ? "Arrived (Default)" : "Timeout (Default)";
                }
                RobotLog.d("AUTO", "Motif Locked (Final): " + detectedMotif);
            }
            limeServo.setPosition(0.5);
            transitionTo(AutoState.SHOOT_PRELOADS);
        }
    }

    private void runShootPreloads() {
        if (follower.isBusy()) return;
        runShootingLogic(false);
    }

    private void runNavToPreIntake() {
        boolean arrived  = !follower.isBusy() && stateTimer.seconds() > 0.25;
        boolean timedOut = stateTimer.seconds() > NAV_TIMEOUT_SEC;

        if (arrived || timedOut) {
            intakeMotor.setPower(1.0);

            chamberTargetPos -= BACK_TO_INTAKE_TICKS;
            chamberSpinner.setTargetPosition((int) chamberTargetPos);
            chamberSpinner.setPower(1.0);

            intakeSeqStage = 0;
            intakeSeqTimer.reset();

            followShootToPreIntakePath();
            transitionTo(AutoState.INTAKE_DRIVE);
        }
    }

    private void runIntakeDrive() {
        updateIntakeIndexing();

        boolean arrived  = !follower.isBusy() && stateTimer.seconds() > 0.25;
        boolean timedOut = stateTimer.seconds() > 3.0;

        if (arrived || timedOut) {
            transitionTo(AutoState.PICKUP_BALLS);
        }
    }

    private void runPickupBalls() {
        updateIntakeIndexing();

        if (stateTimer.seconds() > PICKUP_TIMEOUT_SEC) {
            followReturnToShootPath();
            flywheelMotor.setVelocity(SHOOT_VELOCITY);
            transitionTo(AutoState.NAV_TO_SHOOT);
        }
    }

    private void runNavToShoot() {
        boolean arrived  = !follower.isBusy() && stateTimer.seconds() > 0.5;
        boolean timedOut = stateTimer.seconds() > NAV_TIMEOUT_SEC;

        if (arrived || timedOut) {
            intakeMotor.setPower(0);
            startSecondShootingPhase();
        }
    }

    private void startSecondShootingPhase() {
        chamberTargetPos += SHOOT_POS_TICKS;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setPower(1.0);

        inSecondShootingPhase = true;
        ballsShot     = 0;
        shootSubState = 0;

        transitionTo(AutoState.ALIGN_AND_SHOOT);
    }

    private void runAlignAndShoot() {
        flywheelMotor.setVelocity(SHOOT_VELOCITY);
        runShootingLogic(true);
    }

    private void runLeaveMark() {
        if (!follower.isBusy()) {
            transitionTo(AutoState.DONE);
        }
    }

    // ===================== SHOOTING =====================

    private void runShootingLogic(boolean isSecondPhase) {
        switch (shootSubState) {
            case 0:
                if (ballsShot > 0) {
                    moveChamberStep();
                }
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
                double pushTime = (ballsShot == 0 && !isSecondPhase)
                        ? ATM_PUSH_TIME_FIRST
                        : ATM_PUSH_TIME_NORMAL;

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
                        // Set motif targets, then navigate to pre-intake
                        setTargetForMotif();
                        transitionTo(AutoState.NAV_TO_PRE_INTAKE);
                    } else {
                        // Done shooting — navigate to leave mark
                        Pose leavePose = (selectedAlliance == Alliance.BLUE)
                                ? LEAVE_MARK_BLUE : LEAVE_MARK_RED;
                        followConstantHeadingPath(shootPose, leavePose, shootPose.getHeading());
                        transitionTo(AutoState.LEAVE_MARK);
                    }
                } else {
                    shootSubState = 0;
                }
                break;

            default:
                break;
        }
    }

    // ===================== INTAKE CHAMBER SEQUENCE =====================

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

    // ===================== TARGET SELECTION =====================

    /**
     * Sets preIntakePose and finalIntakePose based on the detected motif and
     * alliance. Does NOT start any path — path is started by runNavToPreIntake().
     */
    private void setTargetForMotif() {
        if (selectedAlliance == Alliance.BLUE) {
            switch (detectedMotif) {
                case GPP:
                    preIntakePose   = BLUE_INTAKE_GPP;
                    finalIntakePose = BLUE_INTAKE_GPP_END;
                    break;
                case PGP:
                    preIntakePose   = BLUE_INTAKE_PGP;
                    finalIntakePose = BLUE_INTAKE_PGP_END;
                    break;
                case PPG:
                    preIntakePose   = BLUE_INTAKE_PPG;
                    finalIntakePose = BLUE_INTAKE_PPG_END;
                    break;
                default:
                    preIntakePose   = BLUE_INTAKE_GPP;
                    finalIntakePose = BLUE_INTAKE_GPP_END;
                    break;
            }
        } else {
            switch (detectedMotif) {
                case GPP:
                    preIntakePose   = RED_INTAKE_GPP;
                    finalIntakePose = RED_INTAKE_GPP_END;
                    break;
                case PGP:
                    preIntakePose   = RED_INTAKE_PGP;
                    finalIntakePose = RED_INTAKE_PGP_END;
                    break;
                case PPG:
                    preIntakePose   = RED_INTAKE_PPG;
                    finalIntakePose = RED_INTAKE_PPG_END;
                    break;
                default:
                    preIntakePose   = RED_INTAKE_GPP;
                    finalIntakePose = RED_INTAKE_GPP_END;
                    break;
            }
        }

        RobotLog.d("AUTO", "Targets set — Pre=" + preIntakePose
                + " | Final=" + finalIntakePose);
    }

    // ===================== PATH HELPERS =====================

    private void followLinearHeadingPath(Pose start, Pose end) {
        currentPath = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
        follower.followPath(currentPath);
    }

    private void followConstantHeadingPath(Pose start, Pose end, double heading) {
        currentPath = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setConstantHeadingInterpolation(heading)
                .build();
        follower.followPath(currentPath);
    }

    /**
     * Drives from shoot pose to pre-intake pose with no wide arc.
     *
     * Strategy (3 segments):
     *   1. Turn in place at shoot XY: linearHeadingInterpolation from shoot
     *      heading → intake heading. A small nudge in the direction of travel
     *      gives Pedro a non-zero vector so it doesn't produce a curved arc.
     *   2. Strafe straight to the correct Y row (constant intake heading).
     *   3. Strafe straight to preIntakePose (constant intake heading).
     */
    private void followShootToPreIntakePath() {
        double intakeHeading = (selectedAlliance == Alliance.BLUE)
                ? Math.toRadians(180) : Math.toRadians(0);

        // Nudge direction: Blue moves in -X toward intake; Red moves in +X.
        double nudgeX = (selectedAlliance == Alliance.BLUE)
                ? -TURN_NUDGE_INCHES : TURN_NUDGE_INCHES;

        // Pose at shoot XY but with intake heading (tiny nudge so Pedro has a vector)
        Pose shootAtIntakeHeading = new Pose(
                shootPose.getX() + nudgeX,
                shootPose.getY(),
                intakeHeading);

        // Pose at shoot X, preIntake Y — the "lane entry" directly above/below target
        Pose laneEntry = new Pose(
                shootPose.getX() + nudgeX,
                preIntakePose.getY(),
                intakeHeading);

        currentPath = follower.pathBuilder()
                // Segment 1: turn in place (linear heading, tiny XY motion)
                .addPath(new BezierLine(shootPose, shootAtIntakeHeading))
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(), intakeHeading)

                // Segment 2: strafe straight to correct row (constant heading)
                .addPath(new BezierLine(shootAtIntakeHeading, laneEntry))
                .setConstantHeadingInterpolation(intakeHeading)

                // Segment 3: strafe straight to pre-intake (constant heading)
                .addPath(new BezierLine(laneEntry, preIntakePose))
                .setConstantHeadingInterpolation(intakeHeading)
                .build();

        follower.followPath(currentPath, true);
    }

    /**
     * Returns from final intake pose back to shoot pose with no wide arc.
     *
     * Strategy (3 segments — mirror of outbound):
     *   1. Strafe straight back to shoot X along the intake row (constant intake heading).
     *   2. Strafe straight to shoot XY still on intake heading.
     *   3. Turn in place back to shoot heading (linear heading interpolation).
     */
    private void followReturnToShootPath() {
        double intakeHeading = (selectedAlliance == Alliance.BLUE)
                ? Math.toRadians(180) : Math.toRadians(0);
        double shootHeading = shootPose.getHeading();

        // Nudge direction mirrors the outbound nudge
        double nudgeX = (selectedAlliance == Alliance.BLUE)
                ? -TURN_NUDGE_INCHES : TURN_NUDGE_INCHES;

        // Lane exit: shoot X, final-intake Y (still on intake heading)
        Pose laneExit = new Pose(
                shootPose.getX() + nudgeX,
                finalIntakePose.getY(),
                intakeHeading);

        // Intermediate at shoot XY with intake heading (before turning back)
        Pose shootAtIntakeHeading = new Pose(
                shootPose.getX() + nudgeX,
                shootPose.getY(),
                intakeHeading);

        currentPath = follower.pathBuilder()
                // Segment 1: strafe back along intake row to shoot X
                .addPath(new BezierLine(finalIntakePose, laneExit))
                .setConstantHeadingInterpolation(intakeHeading)

                // Segment 2: strafe from lane exit to shoot XY (constant heading)
                .addPath(new BezierLine(laneExit, shootAtIntakeHeading))
                .setConstantHeadingInterpolation(intakeHeading)

                // Segment 3: turn in place back to shoot heading
                .addPath(new BezierLine(shootAtIntakeHeading, shootPose))
                .setLinearHeadingInterpolation(intakeHeading, shootHeading)
                .build();

        follower.followPath(currentPath, true);
    }

    // ===================== UTIL =====================

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
        chamberSpinner.setPower(1.0);
    }

    private void updateTelemetry() {
        telemetry.addData("State",        currentState);
        telemetry.addData("Alliance",     selectedAlliance);
        telemetry.addData("Motif",        detectedMotif);
        telemetry.addData("Decision",     decisionReason);
        telemetry.addData("Flywheel Vel", flywheelMotor.getVelocity());

        if (preIntakePose != null) {
            telemetry.addData("PreIntake", "(%.1f, %.1f, %.1f°)",
                    preIntakePose.getX(),
                    preIntakePose.getY(),
                    Math.toDegrees(preIntakePose.getHeading()));
        }

        if (finalIntakePose != null) {
            telemetry.addData("FinalIntake", "(%.1f, %.1f, %.1f°)",
                    finalIntakePose.getX(),
                    finalIntakePose.getY(),
                    Math.toDegrees(finalIntakePose.getHeading()));
        }

        telemetry.update();
    }
}