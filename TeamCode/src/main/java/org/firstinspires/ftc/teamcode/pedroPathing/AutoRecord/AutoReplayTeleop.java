package org.firstinspires.ftc.teamcode.pedroPathing.AutoRecord;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.GoalTargeter;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

public class AutoReplayTeleop {
    // --- Hardware Declarations (from drive.java) ---
    private DcMotor rightFront, rightBack, leftBack, leftFront;
    private DcMotor intakeMotor;
    private DcMotorEx flywheelMotor;
    private DcMotor chamberSpinner;
    public CRServo artifactTransfer;
    private Servo LimeServo;
    private Servo flywheelReadyLed;

    // --- Subsystems ---
    private Limelight limelight;
    private GoalTargeter goalTargeter;
    private Follower follower;
    private AutoReplay autoReplay;

    // --- Telemetry Manager ---
    private TelemetryManager panelsTelemetry;

    // --- Logic Variables ---
    private boolean intaking = false;
    private double intakeSpeed = 1;
    private boolean flywheeling = false;
    private boolean goalLockEnabled = false;

    // --- Flywheel Velocities ---
    private final double HIGH_VELOCITY = 1350;
    private final double LOW_VELOCITY = 1035;

    // --- Slow Mode Variables ---
    private boolean slowMode = false;
    private double speedMultiplier = 1.0;

    // --- Stepper Variables ---
    private double chamberTargetPos = 0;
    private final double ticksPerStep = 475.06;
    private final double shootPosticks = 100.0;
    private final double superReverseTicks = 30.0;

    // --- Button State Trackers ---
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastX = false;
    private boolean lastLT = false;
    private boolean lastRT = false;
    private boolean lastRSB = false;
    private boolean lastDpadDown = false;
    private boolean lastLB = false;
    private boolean lastRB = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadUp = false;

    private int flywheelMode = 0; // 0: off, 1: high, 2: low
    private int shootState = 0;

    // --- Auto Shoot Variables ---
    private ElapsedTime autoShootTimer = new ElapsedTime();
    private int autoBallsShot = 0;
    private final double CHAMBER_WAIT_MS = 1700;
    private final double ATM_PUSH_TIME_MS = 900;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public AutoReplayTeleop(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void init() {
        // --- Follower & Replay Initialization ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        autoReplay = new AutoReplay(follower, telemetry, gamepad1, gamepad2);
        autoReplay.init();

        // --- Panels Telemetry ---
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // --- Hardware Map (from drive.java) ---
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");
        artifactTransfer = hardwareMap.get(CRServo.class, "ATM");
        flywheelReadyLed = hardwareMap.get(Servo.class, "flywheelLed");
        LimeServo = hardwareMap.get(Servo.class, "axonLime");

        // --- Vision Initialization ---
        limelight = new Limelight();
        limelight.init(hardwareMap);
        limelight.switchPipeline(0);
        goalTargeter = new GoalTargeter(limelight);

        // --- Motor Directions ---
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients pidfNew = new PIDFCoefficients(20.3025, 0, 0, 20.7020);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.5);

        LimeServo.setPosition(0.5);
    }

    public void start() {
        follower.startTeleopDrive();
    }

    public void loop() {
        follower.update();
        autoReplay.update();
        goalTargeter.update();

        // Switch between real gamepads and replayed gamepads
        Gamepad gp1 = autoReplay.IsReplayOn() ? autoReplay.getGamepad1() : gamepad1;
        Gamepad gp2 = autoReplay.IsReplayOn() ? autoReplay.getGamepad2() : gamepad2;

        // =========================================================
        //                     DRIVE LOGIC
        // =========================================================

        if (gp1.right_stick_button && !lastRSB) {
            slowMode = !slowMode;
            if (slowMode) {
                speedMultiplier = 0.40;
                gp1.rumble(200);
            } else {
                speedMultiplier = 1.0;
            }
        }
        lastRSB = gp1.right_stick_button;

        double x = Math.pow(gp1.left_stick_x, 3);
        double y = -Math.pow(gp1.left_stick_y, 3);
        double rz = Math.pow(gp1.right_stick_x, 3);

        if (gp1.dpad_down && !lastDpadDown) {
            goalLockEnabled = !goalLockEnabled;
            if (goalLockEnabled) gp1.rumble(300);
        }
        lastDpadDown = gp1.dpad_down;

        if (goalLockEnabled) {
            x = 0; y = 0;
            if (goalTargeter.hasTarget()) {
                int tagID = goalTargeter.getVisionData().getTagID();
                if (tagID == 20 || tagID == 24) rz = -goalTargeter.getSteeringCorrection();
                else rz = 0;
            } else rz = 0;
        }

        // Only drive motors if not busy with a replay path
        if (!follower.isBusy()) {
            leftBack.setPower(((y - x) + rz) * speedMultiplier);
            leftFront.setPower((y + x + rz) * speedMultiplier);
            rightBack.setPower(((y + x) - rz) * speedMultiplier);
            rightFront.setPower(((y - x) - rz) * speedMultiplier);
        }

        // =========================================================
        //                  LIMELIGHT SERVO LOGIC
        // =========================================================

        if (gp2.dpad_right && !lastDpadRight) LimeServo.setPosition(0.75);
        else if (gp2.dpad_left && !lastDpadLeft) LimeServo.setPosition(0.25);
        else if (gp2.dpad_up && !lastDpadUp) LimeServo.setPosition(0.5);

        lastDpadRight = gp2.dpad_right;
        lastDpadLeft = gp2.dpad_left;
        lastDpadUp = gp2.dpad_up;

        // =========================================================
        //                  FLYWHEEL LOGIC (Toggle Triggers)
        // =========================================================

        boolean ltPressed = gp2.left_trigger > 0.3;
        boolean rtPressed = gp2.right_trigger > 0.3;

        if (ltPressed && !lastLT) {
            if (flywheelMode == 1) flywheelMode = 0;
            else flywheelMode = 1;
        }
        if (rtPressed && !lastRT) {
            if (flywheelMode == 2) flywheelMode = 0;
            else flywheelMode = 2;
        }
        lastLT = ltPressed;
        lastRT = rtPressed;

        if (shootState == 0) {
            if (flywheelMode == 1) {
                flywheelMotor.setVelocity(HIGH_VELOCITY);
                flywheeling = true;
            } else if (flywheelMode == 2) {
                flywheelMotor.setVelocity(LOW_VELOCITY);
                flywheeling = true;
            } else {
                flywheelMotor.setVelocity(0);
                flywheeling = false;
            }
        }

        if (flywheeling) {
            double target = (flywheelMode == 1) ? HIGH_VELOCITY : LOW_VELOCITY;
            if (Math.abs(Math.abs(flywheelMotor.getVelocity()) - target) < 75) {
                if ((System.currentTimeMillis() % 1400) < 700) flywheelReadyLed.setPosition(1.0);
                else flywheelReadyLed.setPosition(0.0);
            } else flywheelReadyLed.setPosition(0.0);
        } else flywheelReadyLed.setPosition(0.0);

        // =========================================================
        //                  OTHER MECHANISMS
        // =========================================================

        if (gp1.left_bumper && !lastLB) {
            intaking = !intaking;
            intakeMotor.setPower(intaking ? intakeSpeed : 0);
            gp1.rumble(200);
        }
        lastLB = gp1.left_bumper;

        if (gp2.right_bumper && !lastRB && shootState == 0) {
            if (flywheelMode == 2) {
                shootState = 1;
                autoBallsShot = 0;
                gp2.rumble(500);
            }
        }
        lastRB = gp2.right_bumper;

        boolean triggerPressed = gp1.right_trigger > 0.1;
        boolean dpadRightPressed = gp1.dpad_right;

        if ((triggerPressed || dpadRightPressed) && shootState == 0) {
            artifactTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
            artifactTransfer.setPower(1);
        } else if (shootState == 0) {
            artifactTransfer.setPower(0);
        }

        if (shootState == 0) {
            if (gp2.a && !lastA) moveChamberStep();
            if (gp2.b && !lastB) { chamberTargetPos += shootPosticks; updateChamber(); }
            if (gp2.y && !lastY) { chamberTargetPos -= 100; updateChamber(); }
            if (gp2.x && !lastX) { chamberTargetPos += superReverseTicks; updateChamber(); }
        }
        lastA = gp2.a; lastB = gp2.b; lastY = gp2.y; lastX = gp2.x;

        // =========================================================
        //                  AUTO SHOOT STATE MACHINE
        // =========================================================
        if (shootState > 0) {
            switch (shootState) {
                case 1:
                    if (autoBallsShot > 0) moveChamberStep();
                    autoShootTimer.reset();
                    shootState = 2;
                    break;
                case 2:
                    if (autoShootTimer.milliseconds() >= CHAMBER_WAIT_MS) {
                        artifactTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
                        artifactTransfer.setPower(1);
                        autoShootTimer.reset();
                        shootState = 3;
                    }
                    break;
                case 3:
                    if (autoShootTimer.milliseconds() >= ATM_PUSH_TIME_MS) {
                        artifactTransfer.setPower(0);
                        autoBallsShot++;
                        if (autoBallsShot >= 3) shootState = 0;
                        else shootState = 1;
                    }
                    break;
            }
        }

        // --- Telemetry Updates ---
        panelsTelemetry.debug("DRIVE MODE", slowMode ? "SLOW" : "FULL");
        panelsTelemetry.debug("Goal Lock", goalLockEnabled ? "ENABLED" : "DISABLED");
        panelsTelemetry.debug("Flywheel Vel", flywheelMotor.getVelocity());
        panelsTelemetry.debug("Auto Shoot", shootState > 0 ? "BALL " + (autoBallsShot + 1) : "IDLE");
        panelsTelemetry.debug("Chamber Pos", chamberSpinner.getCurrentPosition());
        panelsTelemetry.debug("Replaying", autoReplay.IsReplayOn());

        panelsTelemetry.update(telemetry);
    }

    private void moveChamberStep() {
        chamberTargetPos += ticksPerStep;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.72);
    }

    private void updateChamber() {
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.5);
    }

    public void stop() {
        if (leftFront != null) {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }
    }
}
