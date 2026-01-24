package org.firstinspires.ftc.teamcode.pedroPathing.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem for the shooter flywheel.
 */
public class Flywheel {

    private DcMotorEx flywheelMotor;

    public void init(HardwareMap hwMap) {
        flywheelMotor = hwMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Sets the flywheel velocity.
     * @param velocity Ticks per second (e.g., 100 as seen in drive file)
     */
    public void setVelocity(double velocity) {
        flywheelMotor.setVelocity(velocity);
    }

    public void stop() {
        flywheelMotor.setPower(0);
    }

    public double getVelocity() {
        return flywheelMotor.getVelocity();
    }
}