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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.GoalTargeter;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.MotifDetector; //cool2


@Autonomous(name = "Basic Far", group = "Auto")
public class AutoFarBasic extends LinearOpMode {

    // ===================== ALLIANCE SELECTION =====================
    private enum Alliance {
        RED,
        BLUE,
    }

    private Alliance selectedAlliance = Alliance.BLUE; // Default

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
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    // ===================== STATE MACHINE =====================
    private enum AutoState {
        INIT,
        DRIVE_TO_SHOOT, // Renamed from SCAN_MOTIF for clarity, functionality remains
        SHOOT_PRELOADS,
        PARK,           // Renamed from LEAVE_MARK
        DONE,
    }

    private AutoState currentState = AutoState.INIT;

    // ===================== CONFIGURATION =====================
    // UPDATED: Changed from Power to Velocity based on Drive file
    private static final double SHOOT_VELOCITY = 1035;

    private static final double CHAMBER_WAIT = 1.7;
    private static final double ATM_PUSH_TIME_FIRST = 0.9;
    private static final double ATM_PUSH_TIME_NORMAL = 0.9;

    // --- Chamber Stepper Variables ---
    private final double TICKS_PER_STEP = 475.06;
    private double chamberTargetPos = 0;

    // ===================== FIELD COORDINATES =====================

    // --- BLUE COORDINATES ---
    private final Pose BLUE_START = new Pose(20.968, 122.296, Math.toRadians(325));
    private final Pose BLUE_SHOOT = new Pose(52.686, 96.116, Math.toRadians(318));
    // Park (Ascent Zone or Observation) - derived from your LEAVE_MARK
    private final Pose BLUE_PARK = new Pose(48.569, 71.869, Math.toRadians(318)); // Adjust if you want to park deeper

    // --- RED COORDINATES ---
    private final Pose RED_START = new Pose(122.672, 122.457, Math.toRadians(215));
    private final Pose RED_SHOOT = new Pose(88.314, 91.116, Math.toRadians(225));
    // Park
    private final Pose RED_PARK = new Pose(97, 73, Math.toRadians(225));

    // Active Points
    private Pose startPose;
    private Pose shootPose;
    private Pose parkPose;

    private PathChain currentPath;

    // --- VARIABLES ---
    private int ballsShot = 0;
    private int shootSubState = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        // ===================== SELECTION LOOP =====================
        while (!isStarted() && !isStopRequested()) {
            // TOGGLE LOGIC
            if (gamepad1.left_bumper) {
                selectedAlliance = Alliance.RED;
            } else if (gamepad1.right_bumper) {
                selectedAlliance = Alliance.BLUE;
            }

            // Vision Updates
            goalTargeter.update();
            motifDetector.update(goalTargeter.getVisionData());

            // Telemetry-driven Servo positioning during selection
            if (selectedAlliance == Alliance.BLUE) {
                LimeServo.setPosition(0.75);
                telemetry.addData("Selected Alliance", "BLUE");
            } else {
                LimeServo.setPosition(0.25);
                telemetry.addData("Selected Alliance", "RED");
            }

            telemetry.addLine("=== SHOOT AND PARK ONLY ===");
            telemetry.addLine("LB = RED | RB = BLUE");
            telemetry.update();
        }

        // ===================== SETUP BASED ON SELECTION =====================
        if (selectedAlliance == Alliance.BLUE) {
            startPose = BLUE_START;
            shootPose = BLUE_SHOOT;
            parkPose = BLUE_PARK;
        } else {
            startPose = RED_START;
            shootPose = RED_SHOOT;
            parkPose = RED_PARK;
        }

        // Initialize Pathing
        follower.setStartingPose(startPose);
        buildAndFollowPath(startPose, shootPose);

        // Spin up flywheel immediately
        flywheelMotor.setVelocity(SHOOT_VELOCITY);

        runtime.reset();
        currentState = AutoState.DRIVE_TO_SHOOT;
        stateTimer.reset();

        // ===================== RUN LOOP =====================
        while (opModeIsActive()) {
            follower.update();
            goalTargeter.update(); // Keep vision running if needed for telemetry

            // Standard Telemetry
            telemetry.addData("State", currentState);
            telemetry.addData("Shots", ballsShot);
            telemetry.addData("Path Busy", follower.isBusy());
            telemetry.update();

            switch (currentState) {
                case DRIVE_TO_SHOOT:
                    // Wait until we reach the shooting position
                    if (!follower.isBusy()) {
                        // Center camera/servo just in case
                        LimeServo.setPosition(0.5);
                        transitionTo(AutoState.SHOOT_PRELOADS);
                    }
                    break;

                case SHOOT_PRELOADS:
                    runShootingSequence();
                    break;

                case PARK:
                    if (!follower.isBusy()) {
                        transitionTo(AutoState.DONE);
                    }
                    break;

                case DONE:
                    stopAllMechanisms();
                    // follower.breakFollowing(); // Optional: keeps robot holding position if removed
                    break;
            }
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

    private void runShootingSequence() {
        switch (shootSubState) {
            case 0:
                // Only spin the chamber if it's NOT the first ball
                if (ballsShot > 0) {
                    moveChamberStep();
                }
                shootTimer.reset();
                shootSubState = 1;
                break;
            case 1:
                // Wait for chamber to settle
                if (shootTimer.seconds() >= CHAMBER_WAIT) {
                    artifactTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
                    artifactTransfer.setPower(1);
                    shootTimer.reset();
                    shootSubState = 2;
                }
                break;
            case 2:
                // Wait for push
                double pushTime = (ballsShot == 0) ? ATM_PUSH_TIME_FIRST : ATM_PUSH_TIME_NORMAL;
                if (shootTimer.seconds() >= pushTime) {
                    artifactTransfer.setPower(0);
                    shootSubState = 3;
                }
                break;
            case 3:
                ballsShot++;
                // Check if we are done shooting 3 balls
                if (ballsShot >= 3) {
                    flywheelMotor.setVelocity(0); // Stop flywheel

                    // Build path to PARK
                    buildAndFollowPath(shootPose, parkPose);

                    transitionTo(AutoState.PARK);
                } else {
                    // Reset for next shot
                    shootSubState = 0;
                }
                break;
        }
    }

    private void buildAndFollowPath(Pose start, Pose end) {
        currentPath = follower
                .pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
        follower.followPath(currentPath);
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
}