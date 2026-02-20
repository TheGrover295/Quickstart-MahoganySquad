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
    private CRServo artifactTransfer;
    private Servo LimeServo;

    // ===================== TIMING =====================
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    // ===================== STATE MACHINE =====================
    private enum AutoState {
        SCAN_MOTIF,
        SHOOT_RANKING_ORDER,
        LEAVE_MARK,
        DONE,
    }

    private AutoState currentState = AutoState.SCAN_MOTIF;

    // ===================== CONFIGURATION =====================
    private static final double SHOOT_VELOCITY = 1035;
    private static final double CHAMBER_WAIT = 0.7;
    private static final double ATM_PUSH_TIME_FIRST = 0.9;
    private static final double ATM_PUSH_TIME_NORMAL = 0.9;
    private final double TICKS_PER_STEP = 475.06;

    // ===================== FIELD COORDINATES =====================
    private final Pose BLUE_START = new Pose(20.968, 122.296, Math.toRadians(325));
    private final Pose BLUE_SHOOT = new Pose(52.686, 96.116, Math.toRadians(318));
    private final Pose LEAVE_MARK_BLUE = new Pose(48.569, 71.869, Math.toRadians(318));

    private final Pose RED_START = new Pose(122.672, 122.457, Math.toRadians(215));
    private final Pose RED_SHOOT = new Pose(88.314, 91.116, Math.toRadians(225));
    private final Pose LEAVE_MARK_RED = new Pose(97, 73, Math.toRadians(225));

    private Pose startPose, shootPose, leavePose;

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
                case SHOOT_RANKING_ORDER: runShootRankingOrder(); break;
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
        artifactTransfer = hardwareMap.get(CRServo.class, "ATM");
        LimeServo = hardwareMap.get(Servo.class, "axonLime");

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfNew = new PIDFCoefficients(20.3025, 0, 0, 20.7020);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.63);

        limelight = new Limelight();
        limelight.init(hardwareMap);
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
            transitionTo(AutoState.SHOOT_RANKING_ORDER);
        }
    }

    private void calculateShootingOrder() {
        boolean[] slotUsed = new boolean[3];
        for (int i = 0; i < 3; i++) {
            char neededColor = detectedMotif.getColorAtIndex(i);
            shootingOrder[i] = -1; // Default
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

    private void runShootRankingOrder() {
        if (follower.isBusy()) return;

        switch (shootSubState) {
            case 0: // Move to target slot
                int targetSlot = shootingOrder[ballsShot];
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
                double pushTime = (ballsShot == 0) ? ATM_PUSH_TIME_FIRST : ATM_PUSH_TIME_NORMAL;
                if (shootTimer.seconds() >= pushTime) {
                    artifactTransfer.setPower(0);
                    shootSubState = 3;
                }
                break;
            case 3:
                ballsShot++;
                if (ballsShot >= 3) {
                    flywheelMotor.setVelocity(0);
                    buildAndFollowPath(shootPose, leavePose);
                    transitionTo(AutoState.LEAVE_MARK);
                } else {
                    shootSubState = 0;
                }
                break;
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
        artifactTransfer.setPower(0);
    }

    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Motif", detectedMotif);
        telemetry.addData("Balls Shot", ballsShot);
        if (ballsShot < 3) telemetry.addData("Targeting Slot", shootingOrder[ballsShot]);
        telemetry.update();
    }
}