package org.firstinspires.ftc.teamcode.pedroPathing.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Subsystem for controlling a Mecanum drive train.
 * Handles both robot-centric and field-centric driving.
 */
public class MecanumDrive {
    private DcMotorEx leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    private IMU imu;

    /**
     * Initializes the drive motors and IMU using values from Constants.
     *
     * @param hwMap The hardware map from the OpMode.
     */
    public void init(HardwareMap hwMap) {

        // Initialize motors using names from Constants
        leftFrontMotor = hwMap.get(DcMotorEx.class, Constants.driveConstants.leftFrontMotorName);
        leftBackMotor = hwMap.get(DcMotorEx.class, Constants.driveConstants.leftRearMotorName);
        rightFrontMotor = hwMap.get(DcMotorEx.class, Constants.driveConstants.rightFrontMotorName);
        rightBackMotor = hwMap.get(DcMotorEx.class, Constants.driveConstants.rightRearMotorName);

        // Set Directions from Constants
        leftFrontMotor.setDirection(Constants.driveConstants.leftFrontMotorDirection);
        leftBackMotor.setDirection(Constants.driveConstants.leftRearMotorDirection);
        rightFrontMotor.setDirection(Constants.driveConstants.rightFrontMotorDirection);
        rightBackMotor.setDirection(Constants.driveConstants.rightRearMotorDirection);

        // Zero Power Behavior
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setting mode to RUN_WITHOUT_ENCODER for TeleOp responsiveness
        // while allowing setVelocity/setPower usage.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    /**
     * Drives the robot using robot-centric controls.
     *
     * @param forward Forward/Backward power (-1.0 to 1.0)
     * @param strafe  Left/Right power (-1.0 to 1.0)
     * @param rotate  Rotation power (-1.0 to 1.0)
     */
    public void drive(double forward, double strafe, double rotate) {

        // Check if all inputs are effectively zero for active braking
        if (Math.abs(forward) < 0.01 && Math.abs(strafe) < 0.01 && Math.abs(rotate) < 0.01) {
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            return;
        }

        double leftFrontPower = forward + strafe + rotate;
        double leftBackPower = forward - strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double rightBackPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = Constants.driveConstants.maxPower;

        maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightBackPower));

        leftFrontMotor.setPower(maxSpeed * (leftFrontPower / maxPower));
        leftBackMotor.setPower(maxSpeed * (leftBackPower / maxPower));
        rightFrontMotor.setPower(maxSpeed * (rightFrontPower / maxPower));
        rightBackMotor.setPower(maxSpeed * (rightBackPower / maxPower));
    }

    /**
     * Drives the robot using field-centric controls.
     *
     * @param forward Forward/Backward power relative to the field (-1.0 to 1.0)
     * @param strafe  Left/Right power relative to the field (-1.0 to 1.0)
     * @param rotate  Rotation power (-1.0 to 1.0)
     */
    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);
        this.drive(newForward, newStrafe, rotate);
    }

    /**
     * Resets the IMU heading to zero.
     */
    public void resetHeading() {
        imu.resetYaw();
    }

    /**
     * Gets the current robot heading in radians.
     *
     * @return Heading in radians (CCW positive).
     */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}