package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "drive")
public class drive extends LinearOpMode {


        private DcMotor rightFront;
        private DcMotor rightBack;
        private DcMotor leftBack;
        private DcMotor leftFront;

        @Override
        public void runOpMode() {

            double speed;
            double x;
            double y;
            double rz;


            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            rightBack = hardwareMap.get(DcMotor.class, "rightBack");
            leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            leftFront = hardwareMap.get(DcMotor.class, "leftFront");

            // Put initialization blocks here.
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
                    telemetry.update();
                }
            }
        }
}

