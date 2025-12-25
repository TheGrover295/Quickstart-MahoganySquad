package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servoTest")
public class servoTest extends LinearOpMode {
    private Servo ATM;


    @Override
    public void runOpMode() {
        ATM = hardwareMap.get(Servo.class, "ATM");
        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {
                if (gamepad1.a) {
                    ATM.setPosition(1);
                }
            }
        }

    }

}
