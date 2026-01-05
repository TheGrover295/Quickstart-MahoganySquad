package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "drive")
public class drive extends LinearOpMode {

    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor flywheelMotor;
    private boolean flywheelSpinning = false;
    private DcMotor intakeMotor;
    private boolean intaking = false;

    private CRServo chamberSpinner;
    public Servo artifactTransfer;

    static final double MAX_DEGREES = 360;
    static final double MIN_POS = 0;
    static final double MAX_POS = 1.0;

    double currentPos = 0.0;

    // --- AXON / CHAMBER LOGIC VARIABLES ---
    private double stepPower = -0.9;
    private double reverseStepPower = -0.9; // Note: Both are negative. Verify if one should be positive?
    private long stepMs = 60;
    private long reverseStepMs = 420;

    private boolean isReverseStepping = false; // "A" button state
    private boolean isForwardStepping = false; // "B" button state

    private boolean lastB = false;
    private boolean lastA = false;
    private final ElapsedTime stepTimer = new ElapsedTime();
    // ---------------------------------------

    @Override
    public void runOpMode() {
        double speed;
        double x, y, rz;
        double flywheelSpeed = 1;
        double intakeSpeed = 1;

        // 1. Initialize Hardware
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        chamberSpinner = hardwareMap.get(CRServo.class, "chamberSpinner");
        artifactTransfer = hardwareMap.get(Servo.class, "artifactTransfer");

        currentPos = MIN_POS;

        // 2. Set Directions
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        // 3. Set Zero Power Behaviors (Do this ONCE, not in the loop)
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {


                speed = 1;
                x = Math.pow(gamepad1.left_stick_x, 3);
                y = -Math.pow(gamepad1.left_stick_y, 3);
                rz = Math.pow(gamepad1.right_stick_x, 3);


                if (gamepad1.dpad_up) {
                    intakeMotor.setPower(intakeSpeed);
                    intaking = true;
                }
                if (gamepad1.dpad_down) {
                    intakeMotor.setPower(0);
                    intaking = false;
                }


                if (gamepad1.x) {
                    flywheelMotor.setPower(flywheelSpeed);
                    flywheelSpinning = true;
                } else {
                    flywheelMotor.setPower(0);
                    flywheelSpinning = false;
                }


                if (gamepad1.y) {
                    artifactTransfer.setPosition(0.5);
                }

                // ------
                boolean b = gamepad1.b;
                boolean a = gamepad1.a;

                // Button B: Short Step (Forward?)
                if (b && !lastB && !isForwardStepping && !isReverseStepping) {
                    isForwardStepping = true;
                    stepTimer.reset();
                    chamberSpinner.setPower(stepPower);
                }

                // Button A: Long Step (Reverse?)
                if (a && !lastA && !isReverseStepping && !isForwardStepping) {
                    isReverseStepping = true;
                    stepTimer.reset();
                    chamberSpinner.setPower(reverseStepPower);
                }

                // Check Timer for Button B (Forward)
                if (isForwardStepping) {
                    if (stepTimer.milliseconds() >= stepMs) {
                        chamberSpinner.setPower(0);
                        isForwardStepping = false;
                    }
                }

                // Check Timer for Button A (Reverse)
                if (isReverseStepping) {
                    if (stepTimer.milliseconds() >= reverseStepMs) {
                        chamberSpinner.setPower(0);
                        isReverseStepping = false;
                    }
                }

                lastB = b;
                lastA = a;
                // ----------------------

                // --- APPLY DRIVE POWERS ---
                leftBack.setPower(((y - x) + rz) * speed);
                leftFront.setPower((y + x + rz) * speed);
                rightBack.setPower(((y + x) - rz) * speed);
                rightFront.setPower(((y - x) - rz) * speed);

                // --- TELEMETRY ---
                telemetry.addData("x", gamepad1.left_stick_x);
                telemetry.addData("y", gamepad1.left_stick_y);
                telemetry.addData("rx", gamepad1.right_stick_x);
                telemetry.addLine("--- STATUS ---");
                telemetry.addData("Intaking", intaking);
                telemetry.addData("Flywheel", flywheelSpinning);
                telemetry.addData("Action", isReverseStepping ? "REVERSE Step" : (isForwardStepping ? "FORWARD Step" : "Idle"));
                telemetry.update();
            }
        }
    }

    // Unused helper method (kept in case you need it later)
    public void moveServoByDegrees(double degrees) {
        double positionChange = degrees / MAX_DEGREES;
        double newPosition = currentPos + positionChange;
        newPosition = Math.max(MIN_POS, Math.min(MAX_POS, newPosition));
        artifactTransfer.setPosition(newPosition);
        currentPos = newPosition;
    }
}