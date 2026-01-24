package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.GoalTargeter;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.MotifDetector;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.VisionData;

// Pedro Imports
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * 6-Ball Autonomous OpMode with Alliance Selection.
 */
@Autonomous(name = "6-Ball Auto (Motif v4)", group = "Auto")
public class AutonomousMode extends LinearOpMode {

    // ===================== ALLIANCE SELECTION =====================
    private enum Alliance {
        RED, BLUE
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

    // ===================== TIMING =====================
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    // ===================== STATE MACHINE =====================
    private enum AutoState {
        INIT,
        SCAN_MOTIF,
        SHOOT_PRELOADS,
        NAV_TO_BALLS,
        PICKUP_BALLS,
        NAV_TO_SHOOT,
        ALIGN_AND_SHOOT,
        DONE
    }

    private AutoState currentState = AutoState.INIT;

    // ===================== CONFIGURATION =====================
    private static final double SCAN_TIMEOUT_SEC = 2.5;
    private static final double NAV_TIMEOUT_SEC = 5.0;
    private static final double PICKUP_TIMEOUT_SEC = 3.0;

    private static final double SHOOT_SPEED = 0.8;
    private static final double CHAMBER_WAIT = 1.9;
    private static final double ATM_PUSH_TIME_FIRST = 2.3;
    private static final double ATM_PUSH_TIME_NORMAL = 0.9;
    private final double TICKS_PER_STEP = 475.06;
    private double chamberTargetPos = 0;

    // ===================== FIELD COORDINATES =====================

    // BLUE START / SHOOT
    private final Pose BLUE_START = new Pose(62.13, 7.03, Math.toRadians(270));
    private final Pose BLUE_SHOOT = new Pose(62.59, 18.90, Math.toRadians(295));


    // GPP = Left (Bottom on Diagram)
    private final Pose BLUE_INTAKE_GPP = new Pose(48, 36, Math.toRadians(0));
    // PGP = Center
    private final Pose BLUE_INTAKE_PGP = new Pose(48, 60, Math.toRadians(0));
    // PPG = Right (Top on Diagram)
    private final Pose BLUE_INTAKE_PPG = new Pose(48, 84, Math.toRadians(0));

    // RED POINTS (Mirrored Approximation)
    private final Pose RED_START = new Pose(82.0, 137.0, Math.toRadians(90));
    private final Pose RED_SHOOT = new Pose(82.0, 125.0, Math.toRadians(65));
    private final Pose RED_INTAKE_GPP = new Pose(15.0, 132.0, Math.toRadians(180));

    private Pose startPose;
    private Pose shootPose;
    private Pose targetIntakePose;

    private PathChain currentPath;

    // ===================== STATE VARIABLES =====================
    private MotifDetector.Motif detectedMotif = MotifDetector.Motif.UNKNOWN;
    private int ballsShot = 0;
    private int shootSubState = 0;
    private boolean inSecondShootingPhase = false;

    // ===================== MAIN LOOP =====================
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.left_bumper) selectedAlliance = Alliance.RED;
            else if (gamepad1.right_bumper) selectedAlliance = Alliance.BLUE;

            goalTargeter.update();
            motifDetector.update(goalTargeter.getVisionData());

            telemetry.addLine("=== 6-BALL AUTO (READY) ===");
            telemetry.addData("Alliance", selectedAlliance);
            telemetry.addData("Motif (Live)", motifDetector.getDetectedMotif());
            telemetry.addData("Pattern", motifDetector.getPatternDescription());
            telemetry.addLine("Map: GPP=Left(36) | PGP=Center(60) | PPG=Right(84)");
            telemetry.update();
        }

        if (selectedAlliance == Alliance.BLUE) {
            startPose = BLUE_START;
            shootPose = BLUE_SHOOT;
        } else {
            startPose = RED_START;
            shootPose = RED_SHOOT;
        }

        follower.setStartingPose(startPose);
        runtime.reset();
        currentState = AutoState.SCAN_MOTIF;
        stateTimer.reset();

        while (opModeIsActive()) {
            follower.update();
            goalTargeter.update();
            motifDetector.update(goalTargeter.getVisionData());

            switch (currentState) {
                case SCAN_MOTIF:       runScanMotif(); break;
                case SHOOT_PRELOADS:   runShootPreloads(); break;
                case NAV_TO_BALLS:     runNavToBalls(); break;
                case PICKUP_BALLS:     runPickupBalls(); break;
                case NAV_TO_SHOOT:     runNavToShoot(); break;
                case ALIGN_AND_SHOOT:  runAlignAndShoot(); break;
                case DONE:
                    flywheelMotor.setPower(0);
                    intakeMotor.setPower(0);
                    artifactTransfer.setPower(0);
                    follower.breakFollowing();
                    break;
            }
            updateTelemetry();
        }
    }

    // ===================== INITIALIZATION =====================
    private void initHardware() {
        follower = Constants.createFollower(hardwareMap);

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        artifactTransfer = hardwareMap.get(CRServo.class, "ATM");

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
    }

    // ===================== STATE METHODS =====================
    private void runScanMotif() {
        if (motifDetector.hasConfidentDetection()) {
            detectedMotif = motifDetector.getDetectedMotif();
            buildAndFollowPath(startPose, shootPose);
            transitionTo(AutoState.SHOOT_PRELOADS);
            return;
        }

        if (stateTimer.seconds() > SCAN_TIMEOUT_SEC) {
            MotifDetector.Motif lastSeen = motifDetector.getDetectedMotif();
            if (lastSeen != MotifDetector.Motif.UNKNOWN) {
                detectedMotif = lastSeen;
                RobotLog.d("AUTO", "Timeout: Using weak detection: " + detectedMotif);
            } else {
                detectedMotif = MotifDetector.Motif.GPP;
                RobotLog.d("AUTO", "Timeout: No detection, defaulting to GPP");
            }

            buildAndFollowPath(startPose, shootPose);
            transitionTo(AutoState.SHOOT_PRELOADS);
        }
    }

    private void runShootPreloads() {
        if (follower.isBusy()) return;
        flywheelMotor.setPower(SHOOT_SPEED);
        runShootingLogic(false);
    }

    private void runNavToBalls() {
        if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
            transitionTo(AutoState.PICKUP_BALLS);
        } else if (stateTimer.seconds() > NAV_TIMEOUT_SEC) {
            transitionTo(AutoState.PICKUP_BALLS);
        }
    }

    private void runPickupBalls() {
        intakeMotor.setPower(1.0);
        if (stateTimer.seconds() > PICKUP_TIMEOUT_SEC) {
            intakeMotor.setPower(0);
            buildAndFollowPath(targetIntakePose, shootPose);
            transitionTo(AutoState.NAV_TO_SHOOT);
        }
    }

    private void runNavToShoot() {
        if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
            inSecondShootingPhase = true;
            ballsShot = 0;
            shootSubState = 0;
            transitionTo(AutoState.ALIGN_AND_SHOOT);
        } else if (stateTimer.seconds() > NAV_TIMEOUT_SEC) {
            inSecondShootingPhase = true;
            ballsShot = 0;
            shootSubState = 0;
            transitionTo(AutoState.ALIGN_AND_SHOOT);
        }
    }

    private void runAlignAndShoot() {
        flywheelMotor.setPower(SHOOT_SPEED);
        runShootingLogic(true);
    }

    // ===================== SHARED LOGIC =====================

    private void runShootingLogic(boolean isSecondPhase) {
        switch (shootSubState) {
            case 0: // Move Chamber
                moveChamberStep();
                shootTimer.reset();
                shootSubState = 1;
                break;
            case 1: // Wait Chamber
                if (shootTimer.seconds() >= CHAMBER_WAIT) {
                    artifactTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
                    artifactTransfer.setPower(1);
                    shootTimer.reset();
                    shootSubState = 2;
                }
                break;
            case 2: // Wait Push
                double pushTime = (ballsShot == 0 && !isSecondPhase) ? ATM_PUSH_TIME_FIRST : ATM_PUSH_TIME_NORMAL;
                if (shootTimer.seconds() >= pushTime) {
                    artifactTransfer.setPower(0);
                    shootSubState = 3;
                }
                break;
            case 3: // Check
                ballsShot++;
                if (ballsShot >= 3) {
                    flywheelMotor.setPower(0);
                    if (!isSecondPhase) {
                        setTargetForMotif();
                        transitionTo(AutoState.NAV_TO_BALLS);
                    } else {
                        transitionTo(AutoState.DONE);
                    }
                } else {
                    shootSubState = 0;
                }
                break;
        }
    }

    private void setTargetForMotif() {
        if (selectedAlliance == Alliance.BLUE) {
            switch (detectedMotif) {
                // Exact mapping from your prompt
                case GPP: targetIntakePose = BLUE_INTAKE_GPP; break; // Left
                case PGP: targetIntakePose = BLUE_INTAKE_PGP; break; // Center
                case PPG: targetIntakePose = BLUE_INTAKE_PPG; break; // Right
                default:  targetIntakePose = BLUE_INTAKE_PGP; break;
            }
        } else {
            targetIntakePose = RED_INTAKE_GPP;
        }

        RobotLog.d("AUTO", "Motif Target: " + detectedMotif + " -> " + targetIntakePose);
        buildAndFollowPath(shootPose, targetIntakePose);
    }

    private void buildAndFollowPath(Pose start, Pose end) {
        currentPath = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
        follower.followPath(currentPath);
    }

    private void transitionTo(AutoState newState) {
        currentState = newState;
        stateTimer.reset();
    }

    private void moveChamberStep() {
        chamberTargetPos += TICKS_PER_STEP;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setPower(0.6);
    }

    private void updateTelemetry() {
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        telemetry.addData("State", currentState.toString());
        telemetry.addData("Alliance", selectedAlliance.toString());
        telemetry.addLine();
        telemetry.addData("Motif", detectedMotif.toString());
        telemetry.addData("Balls Shot", ballsShot + (inSecondShootingPhase ? "/6" : "/3"));
        telemetry.addLine();
// Pedro Telemetry
        telemetry.addData("X", "%.1f", follower.getPose().getX());
        telemetry.addData("Y", "%.1f", follower.getPose().getY());
        telemetry.addData("H", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Busy", follower.isBusy());

        if (goalTargeter.hasTarget()) {
            telemetry.addData("TX", "%.1f°", goalTargeter.getTx());
        }
        telemetry.update();
    }
}