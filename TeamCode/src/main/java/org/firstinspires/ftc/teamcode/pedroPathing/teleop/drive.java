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
    private DcMotor intakeMotor;

    // Changed to CRServo to support the timed "step" rotation
    private CRServo chamberSpinner;
    public Servo artifactTransfer;

    static final double MAX_DEGREES = 360;
    static final double MIN_POS = 0;
    static final double MAX_POS = 1.0;

    double currentPos = 0.0;

    // --- AXON / CHAMBER LOGIC VARIABLES ---
    private double stepPower = 0.8;
    private long stepMs = 162;
    private boolean stepping = false;
    private boolean lastB = false;
    private final ElapsedTime stepTimer = new ElapsedTime();
    // ---------------------------------------

    @Override
    public void runOpMode() {
        double speed;
        double x;
        double y;
        double rz;
        double flywheelSpeed;
        double intakeSpeed;

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Initialize as CRServo
        chamberSpinner = hardwareMap.get(CRServo.class, "chamberSpinner");
        artifactTransfer = hardwareMap.get(Servo.class, "artifactTransfer");

        currentPos = MIN_POS;

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                speed = 1;
                x = Math.pow(gamepad1.left_stick_x, 3);
                y = -Math.pow(gamepad1.left_stick_y, 3);
                rz = Math.pow(gamepad1.right_stick_x, 3);
                flywheelSpeed = 1;
                intakeSpeed = 1;

                if (gamepad1.dpad_up){
                    intakeMotor.setPower(intakeSpeed);
                }
                if (gamepad1.dpad_down){
                    intakeMotor.setPower(0);
                }


                boolean b = gamepad1.b;
                boolean bPressed = b && !lastB;

                if (bPressed && !stepping) {
                    stepping = true;
                    stepTimer.reset();
                    chamberSpinner.setPower(stepPower);

                    flywheelMotor.setPower(flywheelSpeed);
                }

                if (stepping && stepTimer.milliseconds() >= stepMs) {
                    chamberSpinner.setPower(0);
                    stepping = false;
                }
                lastB = b;


                if (gamepad1.a) {
                    moveServoByDegrees(0.5);
                }
                if (gamepad1.y) {
                    artifactTransfer.setPosition(-0.5);
                }
                if (gamepad1.x)
                {
                    flywheelMotor.setPower(0);
                }

                // drive code
                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                leftBack.setPower(((y - x) + rz) * speed);
                leftFront.setPower((y + x + rz) * speed);
                rightBack.setPower(((y + x) - rz) * speed);
                rightFront.setPower(((y - x) - rz) * speed);

                // telemetry
                telemetry.addData("x", gamepad1.left_stick_x);
                telemetry.addData("y", gamepad1.left_stick_y);
                telemetry.addData("rx", gamepad1.right_stick_x);
                telemetry.addData("Chamber Stepping", stepping);
                telemetry.addData("Servo Position", artifactTransfer.getPosition());

                telemetry.update();
            }
        }
    }

    public void moveServoByDegrees(double degrees){
        // Note: artifactTransfer is still a standard Servo
        double positionChange = degrees / MAX_DEGREES;
        double newPosition = currentPos + positionChange;
        newPosition = Math.max(MIN_POS, Math.min(MAX_POS, newPosition));
        artifactTransfer.setPosition(newPosition);
        currentPos = newPosition;
    }
}