package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "colorTest")
public class colorSensorTest extends LinearOpMode {

    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor flywheelMotor;
    private DcMotor intakeMotor;

    private double chamberTargetPos = 0;
    private double ticksPerStep = 475.06;
    private double tinyReverseTicks = 100.0;
    private double superReverseTicks = 30.0;

    private DcMotor chamberSpinner;

    private boolean lastY = false;
    private boolean lastX = false;
    private boolean lastA = false;
    private boolean lastB = false;
    public Servo artifactTransfer;

    // --- LIGHTS & SENSORS ---
    private ColorSensor colorSensor;
    private Servo led;
    private boolean lastArtifactDetected = false;

    static final double MIN_POS = 0;

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
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");

        artifactTransfer = hardwareMap.get(Servo.class, "artifactTransfer");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true); // LED on sensor must be ON to see in dark chamber

        led = hardwareMap.get(Servo.class, "led");

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

                // 1. Get Color Values
                int red = colorSensor.red();
                int green = colorSensor.green();
                int blue = colorSensor.blue();

                // 2. Logic for Specific Colors (Ignoring Alpha/Reflection)
                // Purple is typically High Red + High Blue + Low Green
                boolean isPurple = (red > green) && (blue > green);

                // Yellow is typically High Red + High Green + Low Blue
                boolean isYellow = (red > blue) && (green > blue);

                // Combined detection for robot logic
                boolean artifactDetected = isPurple || isYellow;

                // 3. Control LED Headlight
                led.setPosition(1.0);


                // --- Chamber Manual Overrides (Gamepad 2) ---
                if (gamepad2.a && !lastA) moveChamberStep();
                if (gamepad2.b && !lastB) { chamberTargetPos += tinyReverseTicks; updateChamber(); }
                if (gamepad2.y && !lastY) { chamberTargetPos -= 100; updateChamber(); }
                if (gamepad2.x && !lastX) { chamberTargetPos += superReverseTicks; updateChamber(); }

                lastA = gamepad2.a; lastB = gamepad2.b; lastY = gamepad2.y; lastX = gamepad2.x;
                lastArtifactDetected = artifactDetected;

                // Drivetrain
                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                leftBack.setPower(((y - x) + rz) * speed);
                leftFront.setPower((y + x + rz) * speed);
                rightBack.setPower(((y + x) - rz) * speed);
                rightFront.setPower(((y - x) - rz) * speed);

                // --- TELEMETRY ---
                if (isPurple) {
                    telemetry.addLine("I SEE PURPLE");
                } else if (isYellow) {
                    telemetry.addData("Status", "Yellow Detected");
                } else {
                    telemetry.addData("Status", "Empty / Unknown");
                }

                // Debugging numbers if needed
                telemetry.addData("R", red);
                telemetry.addData("G", green);
                telemetry.addData("B", blue);
                telemetry.update();
            }
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