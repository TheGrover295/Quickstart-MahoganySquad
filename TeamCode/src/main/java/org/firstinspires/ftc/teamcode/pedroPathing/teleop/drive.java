package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.skeletonarmy.marrow.TimerEx; 
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "drive")
public class drive extends LinearOpMode {

    // === 硬件定义 ===
    private DcMotor rightFront, rightBack, leftBack, leftFront;
    private DcMotor flywheelMotor, intakeMotor;
    private DcMotor chamberSpinner;
    public CRServo artifactTransfer;
    private IMU imu;

    // === Marrow 自动功能变量 ===
    private TimerEx autoEjectTimer = new TimerEx(TimeUnit.MILLISECONDS);
    private boolean isAutoEjecting = false;

    // === 逻辑变量 ===
    private boolean intaking = false;
    private double intakeSpeed = 1.0;
    private boolean slowMode = false;
    private double speedMultiplier = 1.0;

    // === 步进电机变量 ===
    private double chamberTargetPos = 0;
    private double ticksPerStep = 475.06;
    private double tinyReverseTicks = 100.0;
    private double superReverseTicks = 30.0;

    // === 按键状态记录 ===
    private boolean lastA = false, lastB = false, lastY = false, lastX = false;

    private GamepadEx operatorOp;
    private GamepadEx driverOp;

    @Override
    public void runOpMode() {
        driverOp = new GamepadEx(gamepad1);
        operatorOp = new GamepadEx(gamepad2);

        // 1. 初始化马达
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");
        artifactTransfer = hardwareMap.get(CRServo.class, "artifactTransfer");

        // 2. 初始化 IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // 3. 马达方向设置
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        // 4. 设置刹车模式
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 5. 步进电机初始设置
        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.5);

        telemetry.addLine("Ready. Spatial Awareness Active.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driverOp.readButtons();
            operatorOp.readButtons();

            if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                slowMode = !slowMode;
                speedMultiplier = slowMode ? 0.25 : 1.0;
                if (slowMode) gamepad1.rumble(200);
            }

            if (gamepad1.options) {
                imu.resetYaw();
                gamepad1.rumble(500);
            }

            // === 空间感知驾驶计算 (Field Centric) ===
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double botHeading = orientation.getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // === 飞轮控制 ===
            if (gamepad2.right_bumper) {
                flywheelMotor.setPower(0.75);
            } else {
                flywheelMotor.setPower(0);
            }

            // === 吸球控制 ===
            if (operatorOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                intaking = !intaking;
                intakeMotor.setPower(intaking ? intakeSpeed : 0);
            }

            // === Marrow 自动吐球 ===
            if (gamepad1.right_trigger > 0.1 && !isAutoEjecting) {
                isAutoEjecting = true;
                autoEjectTimer.start();
                gamepad1.rumble(500);
            }

            if (isAutoEjecting) {
                artifactTransfer.setPower(1.0);
                if (autoEjectTimer.getElapsed() > 1500) { 
                    artifactTransfer.setPower(0);
                    isAutoEjecting = false;
                }
            } else {
                artifactTransfer.setPower(0);
            }

            // === 腔室控制 ===
            if (gamepad2.a && !lastA) moveChamberStep();
            if (gamepad2.b && !lastB) { chamberTargetPos += tinyReverseTicks; updateChamber(); }
            if (gamepad2.y && !lastY) { chamberTargetPos -= 100; updateChamber(); }
            if (gamepad2.x && !lastX) { chamberTargetPos += superReverseTicks; updateChamber(); }

            lastA = gamepad2.a; lastB = gamepad2.b; lastY = gamepad2.y; lastX = gamepad2.x;

            leftFront.setPower(frontLeftPower * speedMultiplier);
            leftBack.setPower(backLeftPower * speedMultiplier);
            rightFront.setPower(frontRightPower * speedMultiplier);
            rightBack.setPower(backRightPower * speedMultiplier);

            telemetry.addData("模式", "Field Centric");
            telemetry.addData("车头朝向", String.format("%.2f 度", Math.toDegrees(botHeading)));
            telemetry.update();
        }
    }

    private void moveChamberStep() {
        chamberTargetPos += ticksPerStep;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.6);
    }

    private void updateChamber() {
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.5);
    }
}
