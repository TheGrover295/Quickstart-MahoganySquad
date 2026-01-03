package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "chamberTest°", group = "Test")
public class chamberTes extends OpMode {

    private CRServo axon;

    // ====== TUNING PARAMETERS ======
    private double stepPower = 0.8; // Speed (0 to 1)
    // To get 1/3 rotation (120°), you likely need to increase this time.
    // If it doesn't go far enough, increase this number. If it goes too far, decrease it.
    private long stepMs = 162;
    // ===============================

    private boolean lastB = false;
    private boolean stepping = false;

    private final ElapsedTime stepTimer = new ElapsedTime();

    @Override
    public void init() {
        axon = hardwareMap.get(CRServo.class, "chamberSpinner");
        axon.setPower(0);
    }

    @Override
    public void loop() {
        boolean b = gamepad1.b;
        boolean bPressed = b && !lastB; // Triggers only once per press

        // If B is pressed and we aren't already moving
        if (bPressed && !stepping) {
            startStep(stepPower); // Positive power moves it FORWARD
        }

        // Stop the servo once the specific time has passed
        if (stepping && stepTimer.milliseconds() >= stepMs) {
            axon.setPower(0);
            stepping = false;
        }

        telemetry.addData("Status", stepping ? "Moving 1/3 Turn" : "Stationary");
        telemetry.addData("Timer", stepTimer.milliseconds());
        telemetry.update();

        lastB = b;
    }

    private void startStep(double power) {
        stepping = true;
        stepTimer.reset();
        axon.setPower(power);
    }
}