package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
// Removed duplicate CRServo import
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@TeleOp(name = "drive")
public class drive extends LinearOpMode {

    private DcMotor rightFront, rightBack, leftBack, leftFront;
    private DcMotor flywheelMotor, intakeMotor;
    private DcMotor chamberSpinner;
    public CRServo artifactTransfer;

    // Logic Variables
    private boolean flywheelSpinning = false;
    private boolean intaking = false;

    private boolean flywheeling = false;
    private double intakeSpeed = 1;

    // Slow Mode Variables
    private boolean slowMode = false;
    private double speedMultiplier = 1.0;

    // Stepper Variables
    private double chamberTargetPos = 0;
    private double ticksPerStep = 475.06; //475.06
    private double tinyReverseTicks = 100.0; //100.0
    private double superReverseTicks = 30.0; //30.0

    // Button State Trackers
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastX = false;

    // Macro Variables
    private int shootState = 0;

    private GamepadEx operatorOp;
    private GamepadEx driverOp;

    @Override
    public void runOpMode() {
        driverOp = new GamepadEx(gamepad1);
        operatorOp = new GamepadEx(gamepad2);

        double x, y, rz;

        // 1. Hardware Map
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");

        artifactTransfer = hardwareMap.get(CRServo.class, "ATM");

        // Directions
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        // Zero Power Behavior
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Chamber Setup
        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.5);

        telemetry.addLine("Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driverOp.readButtons();
            operatorOp.readButtons();

            // --- Slow Mode Logic ---
            if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                slowMode = !slowMode;
                if (slowMode) {
                    speedMultiplier = 0.25;
                    gamepad1.rumble(200);
                    gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
                } else {
                    speedMultiplier = 1.0;
                    gamepad1.setLedColor(255, 0, 0, 1000);
                }
            }

            // --- Drive Input ---
            x = Math.pow(gamepad1.left_stick_x, 3);
            y = -Math.pow(gamepad1.left_stick_y, 3);
            rz = Math.pow(gamepad1.right_stick_x, 3);

            // --- Flywheel Logic (Fixed) ---
            // Left Trigger: 0.75 Power
            // Right Trigger: 0.63 Power
            // Else: 0 Power
            if (gamepad2.left_trigger > 0.1 && shootState == 0) {
                flywheelMotor.setPower(0.8); //0.76
                gamepad2.rumble(200);
                gamepad1.setLedColor(255, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
                flywheeling = true;
            }
            else if (gamepad2.right_trigger > 0.1 && shootState == 0) {
                flywheelMotor.setPower(0.65); // 0.63
                gamepad2.rumble(200);
                gamepad1.setLedColor(255, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
                flywheeling = true;
            }
            else if (shootState == 0) {
                flywheelMotor.setPower(0);
                flywheeling = false;
            }

            // --- Intake Logic ---
            if (operatorOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                intaking = !intaking;
                intakeMotor.setPower(intaking ? intakeSpeed : 0);
                gamepad2.rumble(200);
                gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            }

            // --- ATM / CRServo Logic ---
            boolean triggerPressed = gamepad1.right_trigger > 0.1;
            boolean dpadRightPressed = gamepad1.dpad_right;

            if ((triggerPressed || dpadRightPressed) && shootState == 0) {
                artifactTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
                artifactTransfer.setPower(1);
                gamepad1.rumble(150);
            } else if (shootState == 0) {
                artifactTransfer.setPower(0);
            }

            // --- Chamber Logic ---
            if (gamepad2.a && !lastA) moveChamberStep();
            if (gamepad2.b && !lastB) { chamberTargetPos += tinyReverseTicks; updateChamber(); }
            if (gamepad2.y && !lastY) { chamberTargetPos -= 100; updateChamber(); } //
            if (gamepad2.x && !lastX) { chamberTargetPos += superReverseTicks; updateChamber(); }

            lastA = gamepad2.a; lastB = gamepad2.b; lastY = gamepad2.y; lastX = gamepad2.x;

            // --- Motor Power Application ---
            leftBack.setPower(((y - x) + rz) * speedMultiplier);
            leftFront.setPower((y + x + rz) * speedMultiplier);
            rightBack.setPower(((y + x) - rz) * speedMultiplier);
            rightFront.setPower(((y - x) - rz) * speedMultiplier);

            // --- Telemetry ---
            telemetry.addData("DRIVE MODE", slowMode ? "SLOW (25%)" : "FULL POWER");
            telemetry.addData("Flywheel Spinning", flywheeling);
            telemetry.addData("Chamber Pos", chamberSpinner.getCurrentPosition());
            telemetry.addData("Speed Multiplier", speedMultiplier);
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