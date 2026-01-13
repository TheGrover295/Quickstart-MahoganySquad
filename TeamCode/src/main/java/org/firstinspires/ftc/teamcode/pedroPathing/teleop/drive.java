package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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

    // Slow Mode Variables
    private boolean slowMode = false;
    private double speedMultiplier = 1.0;

    // Stepper Variables
    private double chamberTargetPos = 0;
    private double ticksPerStep = 475.06;
    private double tinyReverseTicks = 100.0;
    private double superReverseTicks = 30.0;

    // Servo Constants
    private final double SERVO_REST = 0.55;
    private final double SERVO_PUSH = 0.7;

    // Button State Trackers
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastX = false;

    // Macro Variables
    private ElapsedTime macroTimer = new ElapsedTime();
    private int macroState = 0;
    private ElapsedTime shootTimer = new ElapsedTime();
    private int shootState = 0;
    private int shotsFired = 0;

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
        artifactTransfer = hardwareMap.get(Servo.class, "ATM");

        // 2. Motor Directions
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        // 3. Zero Power Behaviors
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 4. Chamber Setup
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

            // --- SLOW MODE TOGGLE (DPAD DOWN) ---
            if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                slowMode = !slowMode;
                if (slowMode) {
                    speedMultiplier = 0.25; // 40% speed for precision
                    gamepad1.rumble(200);
                    gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
                } else {
                    speedMultiplier = 1.0; // 100% speed
                    gamepad1.rumble(100);
                    gamepad1.setLedColor(255, 0, 0, 1000); // Flash red then off
                }
            }

            // Drive Input
            x = Math.pow(gamepad1.left_stick_x, 3);
            y = -Math.pow(gamepad1.left_stick_y, 3);
            rz = Math.pow(gamepad1.right_stick_x, 3);

            // --- Flywheel Logic ---
            if (gamepad2.right_bumper && shootState == 0) {
                flywheelMotor.setPower(0.63);
                gamepad2.rumble(200);
                gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            } else if (shootState == 0) {
                flywheelMotor.setPower(0);
                gamepad2.rumble(150);
            }

            // --- Intake Logic ---
            if (operatorOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                intaking = !intaking;
                intakeMotor.setPower(intaking ? intakeSpeed : 0);
                gamepad2.rumble(200);
                gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            }

            // --- ATM / Servo Logic ---
            boolean triggerPressed = gamepad1.right_trigger > 0.1;
            boolean dpadRightPressed = gamepad1.dpad_right;

            if ((triggerPressed || dpadRightPressed) && shootState == 0) {
                artifactTransfer.setDirection(Servo.Direction.REVERSE);
                artifactTransfer.setPosition(SERVO_PUSH);
                gamepad1.rumble(150);

            } else if (shootState == 0) {
                artifactTransfer.setDirection(Servo.Direction.FORWARD);
                artifactTransfer.setPosition(SERVO_REST);
                gamepad1.rumble(50);
            }

            // --- Chamber Manual Overrides (Gamepad 2) ---
            if (gamepad2.a && !lastA) moveChamberStep() ;
            if (gamepad2.b && !lastB) { chamberTargetPos += tinyReverseTicks; updateChamber(); gamepad2.rumble(50);}
            if (gamepad2.y && !lastY) { chamberTargetPos -= 100; updateChamber(); gamepad2.rumble(50);}
            if (gamepad2.x && !lastX) { chamberTargetPos += superReverseTicks; updateChamber(); gamepad2.rumble(50); }

            lastA = gamepad2.a; lastB = gamepad2.b; lastY = gamepad2.y; lastX = gamepad2.x;

            // --- Final Drive Power (Applied Multiplier) ---
            leftBack.setPower(((y - x) + rz) * speedMultiplier);
            leftFront.setPower((y + x + rz) * speedMultiplier);
            rightBack.setPower(((y + x) - rz) * speedMultiplier);
            rightFront.setPower(((y - x) - rz) * speedMultiplier);

            // Telemetry
            telemetry.addData("DRIVE MODE", slowMode ? "SLOW (40%)" : "FULL POWER");
            telemetry.addData("Speed Multiplier", speedMultiplier);
            telemetry.addData("Intaking", intaking);
            telemetry.update();
        }
    }

    private void moveChamberStep() {
        chamberTargetPos += ticksPerStep;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.6);
        gamepad2.rumble(50);
        gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
    }

    private void updateChamber() {
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.5);
        gamepad2.rumble(50);
        gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
    }
}