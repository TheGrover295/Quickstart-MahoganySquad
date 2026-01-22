package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients; // Added for PIDF control
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@TeleOp(name = "drive")
public class tests2 extends LinearOpMode {

    // --- Hardware Declarations ---
    private DcMotorEx launcher;
    private DcMotor left_drive;
    private DcMotor right_drive;
    private CRServo left_feeder;
    private CRServo right_feeder;

    // --- Constants & Variables ---
    // Enums replace the "set IDLE to IDLE" block logic for cleaner Java code
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    private LaunchState launchState = LaunchState.IDLE;
    private double LAUNCHER_TARGET_VELOCITY = 1125;
    private double LAUNCHER_MIN_VELOCITY = 1075;
    private ElapsedTime launchTime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // --- Init Motors (Hardware Mapping & Configuration) ---
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        left_feeder = hardwareMap.get(CRServo.class, "left_feeder");
        right_feeder = hardwareMap.get(CRServo.class, "right_feeder");

        // Launcher Configuration
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set PIDF Coefficients (P=300, I=0, D=0, F=10)
        PIDFCoefficients pidfNew = new PIDFCoefficients(300, 0, 0, 10);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // Drive Configuration
        left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_drive.setDirection(DcMotorSimple.Direction.FORWARD); // Default
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Feeder Configuration
        left_feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        left_feeder.setPower(0);
        right_feeder.setPower(0);

        // Initialize Variables
        launchState = LaunchState.IDLE;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Manual Launcher Control (Y / B) ---
            if (gamepad1.y) {
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            } else if (gamepad1.b) {
                launcher.setVelocity(0);
            }

            // --- Launcher State Machine ---
            boolean shotRequested = gamepad1.right_bumper;

            if (launchState == LaunchState.IDLE) {
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
            }
            else if (launchState == LaunchState.SPIN_UP) {
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                // Check if we reached minimum velocity
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
            }
            else if (launchState == LaunchState.LAUNCH) {
                left_feeder.setPower(1);
                right_feeder.setPower(1);
                launchTime.reset();
                launchState = LaunchState.LAUNCHING;
            }
            else if (launchState == LaunchState.LAUNCHING) {
                if (launchTime.seconds() > 0.2) {
                    left_feeder.setPower(0);
                    right_feeder.setPower(0);
                    launchState = LaunchState.IDLE;
                }
            }

            // --- Telemetry ---
            telemetry.addData("Launch State", launchState);
            telemetry.addData("Launcher Motor Velocity", launcher.getVelocity());
            telemetry.addData("Launch Time", launchTime.seconds());
            telemetry.update();
        }
    }
}