package org.firstinspires.ftc.teamcode.pedroPathing.AutoRecord;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.MecanumDrive;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

/**
 * Simplified TeleOp for recording and replaying paths.
 * 
 * Controls:
 * - Gamepad 1 Double-Tap A: Start/Stop Recording
 * - Gamepad 1 Double-Tap B: Start/Stop Replaying
 * - Gamepad 1 Hold Left Bumper: Select Recording Slot (Log Pointer)
 * - Gamepad 1 Right Stick Button: Toggle Slow Mode
 */
@TeleOp(name = "Auto Replay Teleop", group = "TeleOp")
public class AutoReplayTeleop extends OpMode {
    // --- Subsystems ---
    private Follower follower;
    private AutoReplay autoReplay;
    private MecanumDrive mecanumDrive;

    // --- Telemetry Manager ---
    private TelemetryManager panelsTelemetry;

    // --- State Variables ---
    private boolean slowMode = false;
    private double speedMultiplier = 1.0;
    private boolean lastRSB = false;

    @Override
    public void init() {
        // --- Follower & Replay Initialization ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        autoReplay = new AutoReplay(follower, telemetry, gamepad1, gamepad2);
        autoReplay.init();

        // --- Mecanum Drive Initialization ---
        mecanumDrive = new MecanumDrive();
        mecanumDrive.init(hardwareMap);

        // --- Panels Telemetry ---
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Essential Updates
        follower.update();
        autoReplay.update();

        // Switch between real gamepad and replayed gamepad for drive logic
        Gamepad gp1 = autoReplay.IsReplayOn() ? autoReplay.getGamepad1() : gamepad1;

        // =========================================================
        //                     DRIVE LOGIC
        // =========================================================

        // Slow Mode Toggle
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

        // Drive Inputs
        double x = Math.pow(gp1.left_stick_x, 3);
        double y = -Math.pow(gp1.left_stick_y, 3);
        double rz = Math.pow(gp1.right_stick_x, 3);

        // Manual Drive
        if (!follower.isBusy()) {
            mecanumDrive.drive(y * speedMultiplier, x * speedMultiplier, rz * speedMultiplier);
        }

        // --- Telemetry Updates ---
        panelsTelemetry.debug("DRIVE MODE", slowMode ? "SLOW" : "FULL");
        panelsTelemetry.debug("Replaying", autoReplay.IsReplayOn());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        // Cleanup handled by SDK
    }
}
