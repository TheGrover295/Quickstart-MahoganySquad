package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;



@TeleOp(name = "drive")
public class drive extends LinearOpMode {

    private DcMotor rightFront, rightBack, leftBack, leftFront;
    private DcMotor flywheelMotor, intakeMotor;
    private CRServo chamberSpinner;
    public Servo artifactTransfer;

    // Logic Variables
    private boolean flywheelSpinning = false;
    private boolean intaking = false;
    private double intakeSpeed = 1;
    // Stepper Variables
    private double stepPower = -0.9;
    private double reverseStepPower = -0.9; // Note: Both are negative. Verify if one should be positive?
    private long stepMs = 100; //60 120
    private long reverseStepMs = 480; //420
    private boolean isReverseStepping = false; // "A" button state
    private boolean isForwardStepping = false; // "B" button state
    private boolean lastB = false;
    private boolean lastA = false;
    private final ElapsedTime stepTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        GamepadEx driverOp = new GamepadEx(gamepad1);

        double speed;
        double x, y, rz;

        // ------------------
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        chamberSpinner = hardwareMap.get(CRServo.class, "chamberSpinner");
        artifactTransfer = hardwareMap.get(Servo.class, "artifactTransfer");
        // ------------------
        // 2. Set Directions
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // ------------------
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        // ------------------

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //change
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driverOp.readButtons();
            speed = 1;
            x = Math.pow(gamepad1.left_stick_x, 3);
            y = -Math.pow(gamepad1.left_stick_y, 3);
            rz = Math.pow(gamepad1.right_stick_x, 3);
            // ------------------
            //toggle
            if(gamepad1.right_bumper){
                flywheelMotor.setPower(1);
            }
            // ------------------
            if (driverOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                intaking = !intaking;

                if (intaking) {
                    intakeMotor.setPower(intakeSpeed);
                } else {
                    intakeMotor.setPower(0);
                }
            }
            // ------------------
            if (gamepad1.dpad_right) {
                artifactTransfer.setDirection(Servo.Direction.REVERSE);
                artifactTransfer.setPosition(0.7);
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

            leftBack.setPower(((y - x) + rz) * speed);
            leftFront.setPower((y + x + rz) * speed);
            rightBack.setPower(((y + x) - rz) * speed);
            rightFront.setPower(((y - x) - rz) * speed);

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