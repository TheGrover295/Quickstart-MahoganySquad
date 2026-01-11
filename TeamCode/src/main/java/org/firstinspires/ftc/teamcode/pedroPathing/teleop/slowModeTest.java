package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@TeleOp(name = "slowModeTest")
public class slowModeTest extends LinearOpMode {

    private DcMotor rightFront, rightBack, leftBack, leftFront;

    // Logic Variables
    private boolean slowMode = false;
    private double speedMultiplier = 1.0;

    @Override
    public void runOpMode() {
        GamepadEx driverOp = new GamepadEx(gamepad1);

        double x, y, rz;

        // ------------------
        // 1. Hardware Map
        // ------------------
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        // ------------------
        // 2. Motor Directions
        // ------------------
        // NOTE: Ensure these match your specific robot config
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        // ------------------
        // 3. Zero Power Behaviors (Brakes)
        // ------------------
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driverOp.readButtons();

            // ------------------
            // Slow Mode Logic
            // ------------------
            if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                slowMode = !slowMode;

                // Visual feedback (LEDs/Rumble)
                if (slowMode) {
                    gamepad1.rumble(200);
                    gamepad1.setLedColor(0, 255, 0, 9000);

                } else {
                    gamepad1.rumble(100);
                    gamepad1.setLedColor(255, 0, 0, 9000);

                }
            }


            if (slowMode) {
                speedMultiplier = 0.5; // 50% Speed
            } else {
                speedMultiplier = 1.0; // 100% Speed
            }

            // ------------------
            // Drive Controls
            // ------------------
            x = Math.pow(gamepad1.left_stick_x, 3);
            y = -Math.pow(gamepad1.left_stick_y, 3);
            rz = Math.pow(gamepad1.right_stick_x, 3);

            // Apply multiplier to the final power calculation
            leftBack.setPower(((y - x) + rz) * speedMultiplier);
            leftFront.setPower((y + x + rz) * speedMultiplier);
            rightBack.setPower(((y + x) - rz) * speedMultiplier);
            rightFront.setPower(((y - x) - rz) * speedMultiplier);

            // ------------------
            // Telemetry
            // ------------------
            telemetry.addData("Mode", slowMode ? "SLOW (50%)" : "FAST (100%)");
            telemetry.addData("x", gamepad1.left_stick_x);
            telemetry.addData("y", gamepad1.left_stick_y);
            telemetry.addData("rz", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}