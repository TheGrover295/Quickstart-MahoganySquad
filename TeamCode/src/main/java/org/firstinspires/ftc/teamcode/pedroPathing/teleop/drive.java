package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@TeleOp(name = "drive")
public class drive extends LinearOpMode {

    // --- Hardware Declarations ---
    private DcMotor rightFront, rightBack, leftBack, leftFront;
    private DcMotor intakeMotor;
    private DcMotorEx flywheelMotor;
    private DcMotor chamberSpinner;
    public CRServo artifactTransfer;

    // --- Logic Variables ---
    private boolean intaking = false;
    private double intakeSpeed = 1;
    private boolean flywheeling = false;

    // --- Flywheel Velocities ---
    // Update these values to your preference!
    private double HIGH_VELOCITY = 1365; // 1370
    private double LOW_VELOCITY = 1100;   // Placeholder for lower speed

    // --- Slow Mode Variables ---
    private boolean slowMode = false;
    private double speedMultiplier = 1.0;

    // --- Stepper Variables ---
    private double chamberTargetPos = 0;
    private double ticksPerStep = 475.06;
    private double tinyReverseTicks = 100.0;
    private double superReverseTicks = 30.0;

    // --- Button State Trackers ---
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastX = false;
    private int shootState = 0;

    private GamepadEx operatorOp;
    private GamepadEx driverOp;

    @Override
    public void runOpMode() {
        driverOp = new GamepadEx(gamepad1);
        operatorOp = new GamepadEx(gamepad2);

        // --- Hardware Map ---
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");
        artifactTransfer = hardwareMap.get(CRServo.class, "ATM");

        // --- Motor Configuration ---
        // Drive Motors
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Mechanisms
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        // Flywheel Configuration (Merged with Test Code PIDF)
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Apply PIDF Coefficients from Test Code (P=300, I=0, D=0, F=10)
        PIDFCoefficients pidfNew = new PIDFCoefficients(10, 0, 0, 11); //10p
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // Chamber Setup
        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.5);

        telemetry.addLine("Ready. Left Trigger=High, Right Trigger=Low.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driverOp.readButtons();
            operatorOp.readButtons();

            // =========================================================
            //                     DRIVE LOGIC
            // =========================================================

            // Slow Mode
            if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                slowMode = !slowMode;
                if (slowMode) {
                    speedMultiplier = 0.35;
                    gamepad1.rumble(200);
                    gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
                } else {
                    speedMultiplier = 1.0;
                    gamepad1.setLedColor(255, 0, 0, 1000);
                }
            }

            double x = Math.pow(gamepad1.left_stick_x, 3);
            double y = -Math.pow(gamepad1.left_stick_y, 3);
            double rz = Math.pow(gamepad1.right_stick_x, 3);

            leftBack.setPower(((y - x) + rz) * speedMultiplier);
            leftFront.setPower((y + x + rz) * speedMultiplier);
            rightBack.setPower(((y + x) - rz) * speedMultiplier);
            rightFront.setPower(((y - x) - rz) * speedMultiplier);

            // =========================================================
            //                  FLYWHEEL LOGIC (Split Triggers)
            // =========================================================

            if (shootState == 0) {
                // Left Trigger = High Velocity
                if (gamepad2.left_trigger > 0.1) {
                    flywheelMotor.setVelocity(HIGH_VELOCITY);
                    gamepad2.rumble(100);
                    gamepad1.setLedColor(255, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
                    flywheeling = true;
                }
                // Right Trigger = Low Velocity
                else if (gamepad2.right_trigger > 0.1) {
                    flywheelMotor.setVelocity(LOW_VELOCITY);
                    gamepad2.rumble(220);
                    gamepad1.setLedColor(0, 255, 255, Gamepad.LED_DURATION_CONTINUOUS); // Cyan for Low
                    flywheeling = true;
                }
                // No Trigger = Stop
                else {
                    flywheelMotor.setVelocity(0);
                    flywheeling = false;
                }
            }

            // =========================================================
            //                  OTHER MECHANISMS
            // =========================================================

            // Intake Logic
            if (operatorOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                intaking = !intaking;
                intakeMotor.setPower(intaking ? intakeSpeed : 0);
                gamepad2.rumble(200);
                gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            }

            // Artifact Transfer
            boolean triggerPressed = gamepad1.right_trigger > 0.1;
            boolean dpadRightPressed = gamepad1.dpad_right;

            if ((triggerPressed || dpadRightPressed) && shootState == 0) {
                artifactTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
                artifactTransfer.setPower(1);
                gamepad1.rumble(150);
            } else if (shootState == 0) {
                artifactTransfer.setPower(0);
            }

            // Chamber Logic
            if (gamepad2.a && !lastA) moveChamberStep();
            if (gamepad2.b && !lastB) { chamberTargetPos += tinyReverseTicks; updateChamber(); }
            if (gamepad2.y && !lastY) { chamberTargetPos -= 100; updateChamber(); }
            if (gamepad2.x && !lastX) { chamberTargetPos += superReverseTicks; updateChamber(); }

            lastA = gamepad2.a; lastB = gamepad2.b; lastY = gamepad2.y; lastX = gamepad2.x;

            // =========================================================
            //                       TELEMETRY
            // =========================================================
            telemetry.addData("DRIVE MODE", slowMode ? "SLOW (25%)" : "FULL POWER");
            telemetry.addData("Flywheel Velocity", flywheelMotor.getVelocity());
            if(flywheeling) {
                telemetry.addData("Target", gamepad2.left_trigger > 0.1 ? "HIGH ("+HIGH_VELOCITY+")" : "LOW ("+LOW_VELOCITY+")");
            }
            telemetry.addData("Chamber Pos", chamberSpinner.getCurrentPosition());
            telemetry.addData("Intaking", intaking);
            telemetry.update();
        }
    }

    private void moveChamberStep() {
        chamberTargetPos += ticksPerStep;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.72);
        gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
    }

    private void updateChamber() {
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.5);
        gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
    }
}