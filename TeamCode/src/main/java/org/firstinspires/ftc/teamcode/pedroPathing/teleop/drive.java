package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@TeleOp(name = "drive")
public class drive extends LinearOpMode {

    private DcMotor rightFront, rightBack, leftBack, leftFront;
    private DcMotor flywheelMotor, intakeMotor;
    private DcMotor chamberSpinner;
    public Servo artifactTransfer;

    // Logic Variables
    private boolean flywheelSpinning = false;
    private boolean intaking = false;
    private double intakeSpeed = 1;

    // Stepper Variables
    private double chamberTargetPos = 0;
    private double ticksPerStep = 475.06; // The "Third" of a rotation

    // Button State Trackers
    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void runOpMode() {
        GamepadEx driverOp = new GamepadEx(gamepad1);

        double speed;
        double x, y, rz;

        // ------------------
        // 1. Hardware Map
        // ------------------
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");
        artifactTransfer = hardwareMap.get(Servo.class, "artifactTransfer");

        // ------------------
        // 2. Motor Directions
        // ------------------
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        // OPTIONAL: If the chamber spinner goes the wrong way, uncomment the line below:
        // chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        // ------------------
        // 3. Zero Power Behaviors (Brakes)
        // ------------------
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ------------------
        // 4. Chamber Spinner Setup (Critical for avoiding infinite spin)
        // ------------------
        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // SAFETY: Set power to 0.5 initially so it doesn't spin too fast if broken
        chamberSpinner.setPower(0.5);

        telemetry.addLine("Ready.");
        telemetry.addLine("If Motor Spins Forever: Check Encoder Cable!");
        telemetry.update();
        gamepad1.setLedColor(32, 225, 0, 9000);

        waitForStart();

        while (opModeIsActive()) {
            driverOp.readButtons();

            // Drive Controls
            speed = 1;
            x = Math.pow(gamepad1.left_stick_x, 3);
            y = -Math.pow(gamepad1.left_stick_y, 3);
            rz = Math.pow(gamepad1.right_stick_x, 3);

            // ------------------
            // Flywheel Logic
            if (gamepad1.right_bumper) {
                flywheelMotor.setPower(1);
                gamepad1.rumble(150);
                gamepad1.setLedColor(255, 0, 0, 9000);
            }

            // ------------------
            // Intake Logic
            if (driverOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                intaking = !intaking;
                if (intaking) {
                    intakeMotor.setPower(intakeSpeed);
                    gamepad1.rumble(150);
                    gamepad1.setLedColor(0, 255, 0, 9000);
                } else {
                    intakeMotor.setPower(0);
                }
            }

            // ------------------
            // Artifact Transfer Logic
            if (gamepad1.dpad_right) {
                artifactTransfer.setDirection(Servo.Direction.REVERSE);
                artifactTransfer.setPosition(0.7);
                gamepad1.setLedColor(0, 255, 0, 3000);
            }

            // ==========================================
            // CHAMBER SPINNER LOGIC
            // ==========================================
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            if (a && !lastA) {
                chamberTargetPos += ticksPerStep;
                chamberSpinner.setTargetPosition((int) chamberTargetPos);
                chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                chamberSpinner.setPower(1);
                gamepad1.rumble(200);
                gamepad1.setLedColor(0, 255, 0, 200);
            }

            if (b && !lastB) {
                int currentPos = chamberSpinner.getCurrentPosition();
                chamberTargetPos = currentPos;
                chamberSpinner.setTargetPosition(currentPos);
                gamepad1.rumble(500);
            }

            lastA = a;
            lastB = b;
            // ==========================================

            // Drive Power Calculation
            leftBack.setPower(((y - x) + rz) * speed);
            leftFront.setPower((y + x + rz) * speed);
            rightBack.setPower(((y + x) - rz) * speed);
            rightFront.setPower(((y - x) - rz) * speed);

            // Telemetry for Debugging
            telemetry.addData("x", gamepad1.left_stick_x);
            telemetry.addData("y", gamepad1.left_stick_y);
            telemetry.addData("rx", gamepad1.right_stick_x);
            telemetry.addLine("--- STATUSES ---");
            telemetry.addData("Intaking", intaking);
            telemetry.addData("Flywheel", flywheelSpinning);
            telemetry.addLine("--- CHAMBER DEBUG ---");
            telemetry.addData("Target Position", (int)chamberTargetPos);
            telemetry.addData("Current Position", chamberSpinner.getCurrentPosition());


            telemetry.update();
        }
    }
}