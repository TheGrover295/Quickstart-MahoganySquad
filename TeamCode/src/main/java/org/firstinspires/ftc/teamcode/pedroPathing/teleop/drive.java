package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "drive")
public class drive extends LinearOpMode {


        private DcMotor rightFront;
        private DcMotor rightBack;
        private DcMotor leftBack;
        private DcMotor leftFront;//skib
        private DcMotor flywheelMotor;

        public Servo artifactTransfer;


        static final double MAX_DEGREES = 90;
        static final double MIN_POS = 0; // reprogram servo to be at diffrent 0 pos
        static final double MAX_POS = 1.0;

        double currentPos = 0.0;


    @Override
    public void runOpMode() {
        double speed;
        double x;
        double y;
        double rz;
        double flywheelSpeed;


        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");

        artifactTransfer = hardwareMap.get(Servo.class, "artifactTransfer");
        //artifactTransfer.setPosition(MIN_POS);
        currentPos = MIN_POS;

        // Put initialization blocks here.
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {

                speed = 0.85;
                x = Math.pow(gamepad1.left_stick_x, 3);
                y = -Math.pow(gamepad1.left_stick_y, 3);
                rz = Math.pow(gamepad1.right_stick_x, 3);
                flywheelSpeed = 1;
                //flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                if (gamepad1.a) {
                    moveServoByDegrees(0.5);

                }
                if (gamepad1.y) {
                    artifactTransfer.setPosition(-0.5);
                }
                if (gamepad1.b)
                {
                    flywheelMotor.setPower(flywheelSpeed);
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
                // drive code
                leftBack.setPower(((y - x) + rz) * speed);
                leftFront.setPower((y + x + rz) * speed);
                rightBack.setPower(((y + x) - rz) * speed);
                rightFront.setPower(((y - x) - rz) * speed);
                // telemetry
                telemetry.addData("x", gamepad1.left_stick_x);
                telemetry.addData("y", gamepad1.left_stick_y);
                telemetry.addData("rx", gamepad1.right_stick_x);
                telemetry.addData("Servo Position", artifactTransfer.getPosition());

                telemetry.update();
            }
        }
    }

    public void moveServoByDegrees(double degrees){
        double positionChange = degrees / MAX_DEGREES;
        double newPosition = currentPos + positionChange;

        newPosition = Math.max(MIN_POS, Math.min(MAX_POS, newPosition));

        artifactTransfer.setPosition(newPosition);
        currentPos = newPosition;
    }
}

