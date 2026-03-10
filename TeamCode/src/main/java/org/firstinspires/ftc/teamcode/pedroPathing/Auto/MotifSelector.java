package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

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

@Autonomous(name = "Motif Ranking Auto", group = "Auto")
public class MotifSelector extends LinearOpMode {

    // ===================== ARTIFACT SELECTION =====================
    private enum Artifact {
        GREEN('G'), PURPLE('P');
        char code;
        Artifact(char code) { this.code = code; }
    }

    private Artifact[] chamberSlots = {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};
    private int selectionIndex = 0;
    private boolean lastDpadLeft = false, lastDpadRight = false, lastDpadUpDown = false;

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
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime intakeSeqTimer = new ElapsedTime();

    // ===================== STATE MACHINE =====================
    private enum AutoState {
        SCAN_MOTIF,
        SHOOT_PRELOADS,
        NAV_TO_PRE_INTAKE,
        INTAKE_DRIVE,
        PICKUP_BALLS,
        NAV_TO_SHOOT,
        SHOOT_INTAKE_BALLS,
        LEAVE_MARK,
        DONE,
    }

    private AutoState currentState = AutoState.SCAN_MOTIF;

    // ===================== CONFIGURATION =====================
    private static final double SHOOT_VELOCITY = 1035;
    private static final double CHAMBER_WAIT = 1.7;
    private static final double ATM_PUSH_TIME_FIRST = 0.9;
    private static final double ATM_PUSH_TIME_NORMAL = 0.9;
    private final double TICKS_PER_STEP = 475.06;
    private final double SHOOT_POS_TICKS = 100;
    private final double BACK_TO_INTAKE_TICKS = 100;
    private static final double NAV_TIMEOUT_SEC = 5.0;
    private static final double PICKUP_TIMEOUT_SEC = 2.0;

    // Intake Sequencing Variables
    private int intakeSeqStage = 0;
    private static final double INTAKE_SPIN_DELAY = 0.200;
    private static final double INTAKE_FIRST_DELAY = 3.0;

    // ===================== FIELD COORDINATES =====================
    // --- BLUE COORDINATES ---
    private final Pose BLUE_START = new Pose(20.968, 122.296, Math.toRadians(325));
    private final Pose BLUE_SHOOT = new Pose(52.686, 96.116, Math.toRadians(318));

    // Blue Pre-Intake
    private final Pose BLUE_INTAKE_GPP = new Pose(64, 43, Math.toRadians(-180));
    private final Pose BLUE_INTAKE_PGP = new Pose(59, 67, Math.toRadians(-180));
    private final Pose BLUE_INTAKE_PPG = new Pose(58, 90, Math.toRadians(-180));

    // Blue Intake End
    private final Pose BLUE_INTAKE_GPP_END = new Pose(37, 43, Math.toRadians(-180));
    private final Pose BLUE_INTAKE_PGP_END = new Pose(37, 67, Math.toRadians(-180));
    private final Pose BLUE_INTAKE_PPG_END = new Pose(42, 90, Math.toRadians(-180));

    // --- RED COORDINATES ---
    private final Pose RED_START = new Pose(122.672, 122.457, Math.toRadians(215));
    private final Pose RED_SHOOT = new Pose(88.314, 91.116, Math.toRadians(225));

    // Red Pre-Intake
    private final Pose RED_INTAKE_GPP = new Pose(80, 20, Math.toRadians(0));
    private final Pose RED_INTAKE_PGP = new Pose(84, 42, Math.toRadians(0));
    private final Pose RED_INTAKE_PPG = new Pose(80, 64, Math.toRadians(0));

    // Red Intake End
    private final Pose RED_INTAKE_GPP_END = new Pose(112, 20, Math.toRadians(0));
    private final Pose RED_INTAKE_PGP_END = new Pose(112, 42, Math.toRadians(0));
    private final Pose RED_INTAKE_PPG_END = new Pose(115, 64, Math.toRadians(0));

    private final Pose LEAVE_MARK_RED = new Pose(97, 73, Math.toRadians(225));
    private final Pose LEAVE_MARK_BLUE = new Pose(48.569, 71.869, Math.toRadians(318));

    private Pose startPose, shootPose, leavePose, preIntakePose, finalIntakePose;

    // --- VARIABLES ---
    private MotifDetector.Motif detectedMotif = MotifDetector.Motif.UNKNOWN;
    private int ballsShot = 0;
    private int shootSubState = 0;
    private int currentChamberSlot = 0;
    private int[] shootingOrder = new int[3];
    private double chamberTargetPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        // ===================== SELECTION LOOP =====================
        while (!isStarted() && !isStopRequested()) {
            // Alliance Toggle
            if (gamepad1.left_bumper) selectedAlliance = Alliance.RED;
            if (gamepad1.right_bumper) selectedAlliance = Alliance.BLUE;

            // Artifact Selection
            if (gamepad1.dpad_left && !lastDpadLeft) selectionIndex = (selectionIndex - 1 + 3) % 3;
            if (gamepad1.dpad_right && !lastDpadRight) selectionIndex = (selectionIndex + 1) % 3;
            if ((gamepad1.dpad_up || gamepad1.dpad_down) && !lastDpadUpDown) {
                chamberSlots[selectionIndex] = (chamberSlots[selectionIndex] == Artifact.GREEN) ? Artifact.PURPLE : Artifact.GREEN;
            }
            lastDpadLeft = gamepad1.dpad_left; lastDpadRight = gamepad1.dpad_right; lastDpadUpDown = gamepad1.dpad_up || gamepad1.dpad_down;


            goalTargeter.update();
            motifDetector.update(goalTargeter.getVisionData());

            // Telemetry
            telemetry.addLine("=== MOTIF RANKING AUTO (SELECTION) ===");
            telemetry.addData("Alliance", selectedAlliance + " (LB/RB)");
            telemetry.addLine("\n--- CHAMBER SETUP (DPAD L/R to select, U/D to change) ---");
            for (int i = 0; i < 3; i++) {
                String prefix = (i == selectionIndex) ? "> " : "  ";
                telemetry.addLine(prefix + "Quadrant " + (i+1) + ": " + chamberSlots[i]);
            }
            telemetry.addLine("\nMotif (Preview): " + motifDetector.getDetectedMotif());
            telemetry.update();
        }

        // ===================== START =====================
        if (selectedAlliance == Alliance.BLUE) {
            startPose = BLUE_START; shootPose = BLUE_SHOOT; leavePose = LEAVE_MARK_BLUE;
            LimeServo.setPosition(0.75);
        } else {
            startPose = RED_START; shootPose = RED_SHOOT; leavePose = LEAVE_MARK_RED;
            LimeServo.setPosition(0.25);
        }

        follower.setStartingPose(startPose);
        buildAndFollowPath(startPose, shootPose);
        flywheelMotor.setVelocity(SHOOT_VELOCITY);

        currentState = AutoState.SCAN_MOTIF;
        stateTimer.reset();

        while (opModeIsActive()) {
            follower.update();
            goalTargeter.update();
            motifDetector.update(goalTargeter.getVisionData());

            switch (currentState) {
                case SCAN_MOTIF: runScanMotif(); break;
                case SHOOT_PRELOADS: runShootingLogic(false); break;
                case NAV_TO_PRE_INTAKE: runNavToPreIntake(); break;
                case INTAKE_DRIVE: runIntakeDrive(); break;
                case PICKUP_BALLS: runPickupBalls(); break;
                case NAV_TO_SHOOT: runNavToShoot(); break;
                case SHOOT_INTAKE_BALLS: runShootingLogic(true); break;
                case LEAVE_MARK: runLeaveMark(); break;
                case DONE: stopAllMechanisms(); follower.breakFollowing(); break;
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

    private void runScanMotif() {
        if (detectedMotif == MotifDetector.Motif.UNKNOWN && motifDetector.hasConfidentDetection()) {
            detectedMotif = motifDetector.getDetectedMotif();
        }

        if (!follower.isBusy()) {
            if (detectedMotif == MotifDetector.Motif.UNKNOWN) {
                MotifDetector.Motif lastSeen = motifDetector.getDetectedMotif();
                detectedMotif = (lastSeen != MotifDetector.Motif.UNKNOWN) ? lastSeen : MotifDetector.Motif.GPP;
            }
            LimeServo.setPosition(0.5);
            calculateShootingOrder();
            transitionTo(AutoState.SHOOT_PRELOADS);
        }
    }

    private void calculateShootingOrder() {
        boolean[] slotUsed = new boolean[3];
        for (int i = 0; i < 3; i++) {
            char neededColor = detectedMotif.getColorAtIndex(i);
            shootingOrder[i] = -1; 
            for (int j = 0; j < 3; j++) {
                if (!slotUsed[j] && chamberSlots[j].code == neededColor) {
                    shootingOrder[i] = j;
                    slotUsed[j] = true;
                    break;
                }
            }
            if (shootingOrder[i] == -1) {
                for (int j = 0; j < 3; j++) {
                    if (!slotUsed[j]) {
                        shootingOrder[i] = j;
                        slotUsed[j] = true;
                        break;
                    }
                }
            }
        }
        RobotLog.d("AUTO", "Shooting Order: " + shootingOrder[0] + ", " + shootingOrder[1] + ", " + shootingOrder[2]);
    }

    private void runShootingLogic(boolean isSecondPhase) {
        if (follower.isBusy()) return;
        flywheelMotor.setVelocity(SHOOT_VELOCITY);

        switch (shootSubState) {
            case 0: // Ranking & Spinning Logic
                int targetSlot = shootingOrder[ballsShot];
                
                // If the chamber is wrong, spin it (Ranking Logic)
                int steps = (targetSlot - currentChamberSlot + 3) % 3;
                if (steps > 0) {
                    chamberTargetPos += steps * TICKS_PER_STEP;
                    chamberSpinner.setTargetPosition((int) chamberTargetPos);
                    chamberSpinner.setPower(1);
                }
                currentChamberSlot = targetSlot;

                shootTimer.reset();
                shootSubState = 1;
                break;
            case 1:
                if (shootTimer.seconds() >= CHAMBER_WAIT) {
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
                        buildAndFollowPath(shootPose, leavePose);
                        transitionTo(AutoState.LEAVE_MARK);
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
        buildAndFollowPath(shootPose, preIntakePose);
    }

    private void runNavToPreIntake() {
        if (!follower.isBusy() && stateTimer.seconds() > 0.3) {
            intakeMotor.setPower(1.0);
            chamberTargetPos -= BACK_TO_INTAKE_TICKS;
            chamberSpinner.setTargetPosition((int) chamberTargetPos);
            chamberSpinner.setPower(1);
            intakeSeqStage = 0;
            // Removed intakeSeqTimer.reset() to make the first moveChamberStep() immediate like AutoModeClose
            buildAndFollowPath(preIntakePose, finalIntakePose);
            transitionTo(AutoState.INTAKE_DRIVE);
        } else if (stateTimer.seconds() > NAV_TIMEOUT_SEC) {
            intakeMotor.setPower(1.0);
            chamberTargetPos -= BACK_TO_INTAKE_TICKS;
            chamberSpinner.setTargetPosition((int) chamberTargetPos);
            chamberSpinner.setPower(1);
            intakeSeqStage = 0;
            buildAndFollowPath(preIntakePose, finalIntakePose);
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
            buildAndFollowPath(finalIntakePose, shootPose);
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
            case 2:
            case 3:
                if (intakeSeqTimer.seconds() >= INTAKE_SPIN_DELAY) {
                    moveChamberStep();
                    intakeSeqTimer.reset();
                    intakeSeqStage++;
                }
                break;
        }
    }

    private void moveChamberStep() {
        chamberTargetPos += TICKS_PER_STEP;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setPower(1);
        currentChamberSlot = (currentChamberSlot + 1) % 3;
    }

    private void runNavToShoot() {
        if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
            intakeMotor.setPower(0);
            chamberTargetPos += SHOOT_POS_TICKS;
            chamberSpinner.setTargetPosition((int) chamberTargetPos);
            chamberSpinner.setPower(1);
            
            // Ball 1->sInit, Ball 2->sInit+1, Ball 3->sInit+2
            int sInit = (currentChamberSlot - 4 % 3 + 3) % 3;
            chamberSlots[sInit] = (detectedMotif.getColorAtIndex(0) == 'G') ? Artifact.GREEN : Artifact.PURPLE;
            chamberSlots[(sInit + 1) % 3] = (detectedMotif.getColorAtIndex(1) == 'G') ? Artifact.GREEN : Artifact.PURPLE;
            chamberSlots[(sInit + 2) % 3] = (detectedMotif.getColorAtIndex(2) == 'G') ? Artifact.GREEN : Artifact.PURPLE;
            
            calculateShootingOrder();
            
            ballsShot = 0;
            shootSubState = 0;
            transitionTo(AutoState.SHOOT_INTAKE_BALLS);
        } else if (stateTimer.seconds() > NAV_TIMEOUT_SEC) {
            intakeMotor.setPower(0);
            chamberTargetPos += SHOOT_POS_TICKS;
            chamberSpinner.setTargetPosition((int) chamberTargetPos);
            chamberSpinner.setPower(1);

            int sInit = (currentChamberSlot - 4 % 3 + 3) % 3;
            chamberSlots[sInit] = (detectedMotif.getColorAtIndex(0) == 'G') ? Artifact.GREEN : Artifact.PURPLE;
            chamberSlots[(sInit + 1) % 3] = (detectedMotif.getColorAtIndex(1) == 'G') ? Artifact.GREEN : Artifact.PURPLE;
            chamberSlots[(sInit + 2) % 3] = (detectedMotif.getColorAtIndex(2) == 'G') ? Artifact.GREEN : Artifact.PURPLE;

            calculateShootingOrder();

            ballsShot = 0;
            shootSubState = 0;
            transitionTo(AutoState.SHOOT_INTAKE_BALLS);
        }
    }

    private void runLeaveMark() {
        if (!follower.isBusy()) transitionTo(AutoState.DONE);
    }

    private void buildAndFollowPath(Pose start, Pose end) {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
        follower.followPath(path);
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

    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Motif", detectedMotif);
        telemetry.addData("Balls Shot", ballsShot);
        if (ballsShot < 3 && (currentState == AutoState.SHOOT_PRELOADS || currentState == AutoState.SHOOT_INTAKE_BALLS)) 
            telemetry.addData("Targeting Slot", shootingOrder[ballsShot]);
        telemetry.update();
    }
}