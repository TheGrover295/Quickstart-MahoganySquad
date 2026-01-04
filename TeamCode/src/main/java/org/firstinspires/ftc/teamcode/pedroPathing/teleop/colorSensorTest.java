package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
    private boolean flywheelSpinning = false;
    private DcMotor intakeMotor;
    private boolean intaking = false;

    private CRServo chamberSpinner;
    public Servo artifactTransfer;

    // --- NEW: LIGHTS & SENSORS ---
    private ColorSensor colorSensor;
    private Servo led; // The goBILDA Headlight is controlled like a Servo
    private boolean lastArtifactDetected = false;

    // TUNING: Threshold for "Is there an object?" (0 = Pitch Black, 400+ = Bright Reflection)
    private static final int MIN_LIGHT_THRESHOLD = 150;
    // -----------------------------

    static final double MAX_DEGREES = 360;
    static final double MIN_POS = 0;
    static final double MAX_POS = 1.0;

    double currentPos = 0.0;

    // --- AXON / CHAMBER LOGIC ---
    private double stepPower = 0.8;
    private double reverseStepPower = -0.8;
    private long stepMs = 162;
    private long ReversestepMs = 162;
    private boolean stepping = false;
    private boolean isReversing = false;
    private boolean lastB = false;
    private boolean lastA = false;
    private final ElapsedTime stepTimer = new ElapsedTime();
    // ----------------------------

    @Override
    public void runOpMode() {
        double speed;
        double x;
        double y;
        double rz;
        double flywheelSpeed;
        double intakeSpeed;

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        chamberSpinner = hardwareMap.get(CRServo.class, "chamberSpinner");
        artifactTransfer = hardwareMap.get(Servo.class, "artifactTransfer");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true); // LED on sensor must be ON to see in dark chamber

        // Initialize the goBILDA LED Headlight
        // Make sure it is plugged into a SERVO port and named "led" in config
        led = hardwareMap.get(Servo.class, "led");

        currentPos = MIN_POS;

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
                flywheelSpeed = 1;
                intakeSpeed = 1;

                // -----------
                boolean b = gamepad1.b;
                boolean a = gamepad1.a;

                // 1. Check Color Sensor
                boolean artifactDetected = checkForArtifact();

                // 2. Control LED Headlight
                if (artifactDetected) {
                    led.setPosition(1.0); // Turn Light ON (Max Brightness)
                } else {
                    led.setPosition(0.0); // Turn Light OFF
                }

                // 3. Logic: Trigger spin if B pressed OR (Artifact just appeared)
                boolean autoTrigger = artifactDetected && !lastArtifactDetected;

                if ((b || autoTrigger) && !lastB && !stepping) {
                    stepping = true;
                    isReversing = false;
                    stepTimer.reset();
                    //chamberSpinner.setPower(stepPower);
                    //flywheelMotor.setPower(flywheelSpeed);
                }

                if (a && !lastA && !stepping) {
                    stepping = true;
                    isReversing = true;
                    stepTimer.reset();
                    //chamberSpinner.setPower(reverseStepPower);
                }

                if (stepping) {
                    long targetMs = isReversing ? ReversestepMs : stepMs;
                    if (stepTimer.milliseconds() >= targetMs) {
                        //chamberSpinner.setPower(0);
                        stepping = false;
                    }
                }

                lastB = b;
                lastA = a;
                lastArtifactDetected = artifactDetected;
                // ----------------------

                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                leftBack.setPower(((y - x) + rz) * speed);
                leftFront.setPower((y + x + rz) * speed);
                rightBack.setPower(((y + x) - rz) * speed);
                rightFront.setPower(((y - x) - rz) * speed);

                // Telemetry
                telemetry.addData("Chamber Stepping", stepping);
                telemetry.addData("Artifact Detected", artifactDetected ? "YES" : "NO");
                telemetry.addData("LED Status", artifactDetected ? "ON" : "OFF");
                telemetry.addData("Alpha", colorSensor.alpha());
                telemetry.update();
            }
        }
    }

    private boolean checkForArtifact() {
        // 1. Light Intensity Check (Is it dark/empty?)
        if (colorSensor.alpha() < MIN_LIGHT_THRESHOLD) {
            return false;
        }

        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        // 2. Color Logic
        boolean isGreen = (green > red) && (green > blue);
        boolean isPurple = (red > green) && (blue > green);

        return isGreen || isPurple;
    }

    public void moveServoByDegrees(double degrees){
        double positionChange = degrees / MAX_DEGREES;
        double newPosition = currentPos + positionChange;
        newPosition = Math.max(MIN_POS, Math.min(MAX_POS, newPosition));
        artifactTransfer.setPosition(newPosition);
        currentPos = newPosition;
    }
}