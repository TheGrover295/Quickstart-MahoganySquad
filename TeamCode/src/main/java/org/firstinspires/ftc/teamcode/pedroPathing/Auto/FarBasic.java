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
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FarBasic", group = "Auto")
public class FarBasic extends LinearOpMode {

    // ===================== ALLIANCE SELECTION =====================
    private enum Alliance {
        RED,
        BLUE,
    }

    private Alliance selectedAlliance = Alliance.BLUE; // Default

    // ===================== SUBSYSTEMS =====================
    private Follower follower;
    private DcMotorEx flywheelMotor;
    private DcMotor chamberSpinner;
    private CRServo artifactTransfer;

    // ===================== TIMING =====================
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    // ===================== STATE MACHINE =====================
    private enum AutoState {
        INIT,
        DRIVE_TO_SHOOT,
        SHOOT_PRELOADS,
        LEAVE_MARK,
        DONE,
    }

    private AutoState currentState = AutoState.INIT;

    // ===================== CONFIGURATION =====================
    private static final double SHOOT_VELOCITY = 1280;
    private static final double CHAMBER_WAIT = 1.5;
    private static final double ATM_PUSH_TIME_FIRST = 2.0;
    private static final double ATM_PUSH_TIME_NORMAL = 0.9;

    // --- Chamber Stepper Variables ---
    private final double TICKS_PER_STEP = 475.06;
    private double chamberTargetPos = 0;

    // ===================== FIELD COORDINATES =====================

    private final Pose BLUE_START = new Pose(57, 8.5, Math.toRadians(270));
    private final Pose BLUE_SHOOT = new Pose(56, 15, Math.toRadians(292));
    private final Pose LEAVE_MARK_BLUE = new Pose(37.269, 12.946, Math.toRadians(270));

    private final Pose RED_START = new Pose(87, 8.5, Math.toRadians(270));
    private final Pose RED_SHOOT = new Pose(88, 19, Math.toRadians(250));
    private final Pose LEAVE_MARK_RED = new Pose(107.977, 13.269, Math.toRadians(270));

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
            if (gamepad1.left_bumper) {
                selectedAlliance = Alliance.RED;
            } else if (gamepad1.right_bumper) {
                selectedAlliance = Alliance.BLUE;
            }

            telemetry.addLine("=== FAR BASIC (ALLIANCE SELECTION) ===");
            telemetry.addData("Selected Alliance", selectedAlliance);
            telemetry.addLine("LB = RED | RB = BLUE");
            telemetry.update();
        }

        // ===================== SETUP BASED ON SELECTION =====================
        if (selectedAlliance == Alliance.BLUE) {
            startPose = BLUE_START;
            shootPose = BLUE_SHOOT;
            parkPose = LEAVE_MARK_BLUE;
        } else {
            startPose = RED_START;
            shootPose = RED_SHOOT;
            parkPose = LEAVE_MARK_RED;
        }

        follower.setStartingPose(startPose);
        
        // Start by driving to shoot position
        buildAndFollowPath(startPose, shootPose);
        flywheelMotor.setVelocity(SHOOT_VELOCITY);
        transitionTo(AutoState.DRIVE_TO_SHOOT);

        // ===================== RUN LOOP =====================
        while (opModeIsActive()) {
            follower.update();

            switch (currentState) {
                case DRIVE_TO_SHOOT:
                    if (!follower.isBusy()) {
                        transitionTo(AutoState.SHOOT_PRELOADS);
                    }
                    break;
                case SHOOT_PRELOADS:
                    runShootingLogic();
                    break;
                case LEAVE_MARK:
                    if (!follower.isBusy()) {
                        transitionTo(AutoState.DONE);
                    }
                    break;
                case DONE:
                    stopAllMechanisms();
                    follower.breakFollowing();
                    break;
            }
            updateTelemetry();
        }
    }

    private void initHardware() {
        follower = Constants.createFollower(hardwareMap);
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");
        artifactTransfer = hardwareMap.get(CRServo.class, "ATM");

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients pidfNew = new PIDFCoefficients(11, 0, 0, 10);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.63);
    }

    private void runShootingLogic() {
        switch (shootSubState) {
            case 0:
                // Skip chamber rotation for the first ball
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
                double pushTime = (ballsShot == 0) ? ATM_PUSH_TIME_FIRST : ATM_PUSH_TIME_NORMAL;
                if (shootTimer.seconds() >= pushTime) {
                    artifactTransfer.setPower(0);
                    shootSubState = 3;
                }
                break;
            case 3:
                ballsShot++;
                // Shoot only once (one ball) as requested, or adjust to 3 for all preloads
                if (ballsShot >= 1) { 
                    flywheelMotor.setVelocity(0);
                    buildAndFollowPath(shootPose, parkPose);
                    transitionTo(AutoState.LEAVE_MARK);
                } else {
                    shootSubState = 0;
                }
                break;
        }
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

    private void stopAllMechanisms() {
        flywheelMotor.setVelocity(0);
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
        telemetry.addData("Flywheel Vel", flywheelMotor.getVelocity());
        telemetry.addData("Balls Shot", ballsShot);
        telemetry.update();
    }
}
