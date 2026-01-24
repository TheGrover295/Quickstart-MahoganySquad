package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "ColorTest",  group = "Tests")
public class ColorSensorTest extends OpMode {
    TestBenchColor bench = new TestBenchColor();
    @Override
    public void init(){ // made by WB
        bench.init(hardwareMap);
    }

    @Override
    public void loop(){
        bench.getDetectedColor(telemetry);
    }

}
