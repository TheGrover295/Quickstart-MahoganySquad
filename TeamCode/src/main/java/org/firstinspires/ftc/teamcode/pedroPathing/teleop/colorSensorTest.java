package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "colorTest")
public class colorSensorTest extends LinearOpMode {

    private ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Initialize the color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        
        // LED on sensor should be ON for better detection in closed spaces
        colorSensor.enableLed(true);

        telemetry.addData("Status", "Initialized. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get color values
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Logic for Green and Purple
            // Green: Green channel is significantly higher than red and blue
            boolean isGreen = (green > red) && (green > blue);

            // Purple: Red and Blue are both significantly higher than green
            boolean isPurple = (red > green) && (blue > green);

            // Display results
            if (isGreen) {
                telemetry.addLine("DETECTED: GREEN");
            } else if (isPurple) {
                telemetry.addLine("DETECTED: PURPLE");
            } else {
                telemetry.addLine("DETECTED: NONE / UNKNOWN");
            }

            // Raw data for debugging
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.update();
        }
    }
}
