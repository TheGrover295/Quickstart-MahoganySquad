package org.firstinspires.ftc.teamcode.pedroPathing.AutoRecord;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.io.FileWriter;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class AutoReplay {
    Follower follower;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;

    Gamepad gamepadReplay1 = new Gamepad();
    Gamepad gamepadReplay2 = new Gamepad();

    PressHold recording;
    PressHold replay;
    PressHold pointerInput;
    
    Pose lastPose = new Pose(0, 0, 0);
    double deltaError = 1.0; // In inches
    double deltaHeading = Math.toRadians(5);
    int replayIndex = 0;
    StateEntryJson currentReplayStates;
    PathChain replayPath;
    GamepadStateEntry gamepadDelta1;
    GamepadStateEntry gamepadDelta2;
    int logPointer = 0;

    public AutoReplay(Follower follower, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.follower = follower;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void init() {
        recording = new PressHold(PressHold.PressType.DoublePress);
        replay = new PressHold(PressHold.PressType.DoublePress);
        pointerInput = new PressHold(PressHold.PressType.LongPress);
        currentReplayStates = new StateEntryJson();
        loadPointer();
    }

    public void recordPositions() {
        File dir = new File(AppUtil.ROOT_FOLDER + "/TeamCodeLogs");
        if (!dir.exists()) {
            boolean ignored = dir.mkdirs();
        }
        File file = new File(dir, "movement" + logPointer + ".json");
        try (FileWriter writer = new FileWriter(file)) {
            Gson gson = new GsonBuilder().setPrettyPrinting().create();
            writer.write(gson.toJson(currentReplayStates));
            telemetry.addData("Drive log written", file.getAbsolutePath());
        } catch (Exception e) {
            telemetry.addData("Drive log error", e.getMessage());
            RobotLog.e("AutoReplay: Record Error: " + e.getMessage());
        }
    }

    public void savePointer() {
        File dir = new File(AppUtil.ROOT_FOLDER + "/TeamCodeLogs");
        if (!dir.exists()) {
            boolean ignored = dir.mkdirs();
        }
        File file = new File(dir, "pointer.json");
        try (FileWriter writer = new FileWriter(file)) {
            Gson gson = new GsonBuilder().create();
            writer.write(gson.toJson(new PointerJson(logPointer)));
            telemetry.addData("Pointer Saved", logPointer);
        } catch (Exception e) {
            telemetry.addData("Pointer error", e.getMessage());
        }
    }

    public void loadPoses() {
        File dir = new File(AppUtil.ROOT_FOLDER + "/TeamCodeLogs");
        File file = new File(dir, "movement" + logPointer + ".json");
        if (!file.exists()) return;
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            Gson gson = new GsonBuilder().create();
            currentReplayStates = gson.fromJson(reader, StateEntryJson.class);
        } catch (Exception e) {
            telemetry.addData("Failed to Load", e.getMessage());
        }
    }

    public void loadPointer() {
        File dir = new File(AppUtil.ROOT_FOLDER + "/TeamCodeLogs");
        File file = new File(dir, "pointer.json");
        if (!file.exists()) return;
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            Gson gson = new GsonBuilder().create();
            logPointer = gson.fromJson(reader, PointerJson.class).pointer;
        } catch (Exception e) {}
    }

    public void update() {
        recording.checkStatus(gamepad1.a);
        replay.checkStatus(gamepad1.b);
        pointerInput.checkStatus(gamepad1.left_bumper);

        if (pointerInput.startPress) logPointer = 0;
        if (pointerInput.isOn) logPointer = (int) Math.floor(pointerInput.time.seconds());
        if (pointerInput.endPress) savePointer();

        if (recording.startPress) {
            currentReplayStates = new StateEntryJson();
            lastPose = follower.getPose();
            if (lastPose == null) lastPose = new Pose(0,0,0);
            gamepadDelta1 = new GamepadStateEntry(gamepad1);
            gamepadDelta2 = new GamepadStateEntry(gamepad2);
            addCurrentState();
        }

        if (recording.isOn) {
            Pose currentPose = follower.getPose();
            if (currentPose != null) {
                double dist = Math.hypot(currentPose.getX() - lastPose.getX(), currentPose.getY() - lastPose.getY());
                double headingDiff = Math.abs(currentPose.getHeading() - lastPose.getHeading());
                if (dist > deltaError || headingDiff > deltaHeading) {
                    addCurrentState();
                    lastPose = currentPose;
                    gamepadDelta1 = new GamepadStateEntry(gamepad1);
                    gamepadDelta2 = new GamepadStateEntry(gamepad2);
                } else {
                    gamepadDelta1.mergeBooleans(new GamepadStateEntry(gamepad1));
                    gamepadDelta2.mergeBooleans(new GamepadStateEntry(gamepad2));
                }
            }
        }

        if (recording.endPress) recordPositions();

        if (replay.startPress) {
            loadPoses();
            if (currentReplayStates != null && currentReplayStates.size > 1) {
                ArrayList<Path> paths = new ArrayList<>();
                Pose currentPose = follower.getPose();
                if (currentPose == null) currentPose = new Pose(0,0,0);
                PoseStateEntry startRecorded = currentReplayStates.poseList.get(0);
                paths.add(new Path(new BezierLine(currentPose, new Pose(startRecorded.x, startRecorded.y, startRecorded.heading))));
                for (int i = 0; i < currentReplayStates.size - 1; i++) {
                    PoseStateEntry p1 = currentReplayStates.poseList.get(i);
                    PoseStateEntry p2 = currentReplayStates.poseList.get(i + 1);
                    paths.add(new Path(new BezierLine(new Pose(p1.x, p1.y, p1.heading), new Pose(p2.x, p2.y, p2.heading))));
                }
                replayPath = new PathChain(paths);
                follower.followPath(replayPath, true);
                replayIndex = 0;
            }
        }

        if (replay.isOn) {
            replayIndex = (int) follower.getCurrentPathNumber();
            if (currentReplayStates != null && replayIndex < currentReplayStates.gamepad1List.size()) {
                gamepadReplay1 = currentReplayStates.gamepad1List.get(replayIndex).convertToGamepad();
                gamepadReplay2 = currentReplayStates.gamepad2List.get(replayIndex).convertToGamepad();
            }
        }

        if (replay.endPress) {
            follower.breakFollowing();
            follower.startTeleopDrive();
        }
        displayTelemetry();
    }

    private void addCurrentState() {
        Pose p = follower.getPose();
        if (p == null) p = new Pose(0,0,0);
        currentReplayStates.timeList.add(recording.time.seconds());
        currentReplayStates.poseList.add(new PoseStateEntry(p));
        currentReplayStates.gamepad1List.add(gamepadDelta1);
        currentReplayStates.gamepad2List.add(gamepadDelta2);
        currentReplayStates.size++;
    }

    private void displayTelemetry() {
        telemetry.addData("LOG POINTER", logPointer);
        telemetry.addData("Recording", recording.isOn);
        telemetry.addData("Replaying", replay.isOn);
        if (recording.isOn) telemetry.addData("Recorded States", currentReplayStates.size);
        if (replay.isOn) telemetry.addData("Replay Index", replayIndex + "/" + currentReplayStates.size);
        telemetry.addData("Follower Busy", follower.isBusy());
    }

    public boolean IsReplayOn() {
        return replay != null && replay.isOn;
    }

    public Gamepad getGamepad1() { return gamepadReplay1; }
    public Gamepad getGamepad2() { return gamepadReplay2; }

    public static class GamepadStateEntry {
        public boolean a, b, x, y, dpad_up, dpad_down, dpad_left, dpad_right, left_bumper, right_bumper, left_stick_button, right_stick_button;
        public float left_stick_x, left_stick_y, right_stick_x, right_stick_y, left_trigger, right_trigger;

        public GamepadStateEntry(Gamepad g) {
            this.a = g.a; this.b = g.b; this.x = g.x; this.y = g.y;
            this.dpad_up = g.dpad_up; this.dpad_down = g.dpad_down; this.dpad_left = g.dpad_left; this.dpad_right = g.dpad_right;
            this.left_bumper = g.left_bumper; this.right_bumper = g.right_bumper;
            this.left_stick_button = g.left_stick_button; this.right_stick_button = g.right_stick_button;
            this.left_stick_x = g.left_stick_x; this.left_stick_y = g.left_stick_y;
            this.right_stick_x = g.right_stick_x; this.right_stick_y = g.right_stick_y;
            this.left_trigger = g.left_trigger; this.right_trigger = g.right_trigger;
        }

        public Gamepad convertToGamepad() {
            Gamepad g = new Gamepad();
            g.a = this.a; g.b = this.b; g.x = this.x; g.y = this.y;
            g.dpad_up = this.dpad_up; g.dpad_down = this.dpad_down; g.dpad_left = this.dpad_left; g.dpad_right = this.dpad_right;
            g.left_bumper = this.left_bumper; g.right_bumper = this.right_bumper;
            g.left_stick_button = this.left_stick_button; g.right_stick_button = this.right_stick_button;
            g.left_stick_x = this.left_stick_x; g.left_stick_y = this.left_stick_y;
            g.right_stick_x = this.right_stick_x; g.right_stick_y = this.right_stick_y;
            g.left_trigger = this.left_trigger; g.right_trigger = this.right_trigger;
            return g;
        }

        public void mergeBooleans(GamepadStateEntry other) {
            this.a |= other.a; this.b |= other.b; this.x |= other.x; this.y |= other.y;
            this.dpad_up |= other.dpad_up; this.dpad_down |= other.dpad_down;
            this.dpad_left |= other.dpad_left; this.dpad_right |= other.dpad_right;
            this.left_bumper |= other.left_bumper; this.right_bumper |= other.right_bumper;
            this.left_stick_button |= other.left_stick_button; this.right_stick_button |= other.right_stick_button;
        }
    }

    public static class PoseStateEntry {
        public double x, y, heading;
        public PoseStateEntry(Pose pose) {
            this.x = pose.getX(); this.y = pose.getY(); this.heading = pose.getHeading();
        }
    }

    public static class StateEntryJson {
        public int size = 0;
        public List<Double> timeList = new ArrayList<>();
        public List<PoseStateEntry> poseList = new ArrayList<>();
        public List<GamepadStateEntry> gamepad1List = new ArrayList<>();
        public List<GamepadStateEntry> gamepad2List = new ArrayList<>();
    }

    public static class PointerJson {
        public int pointer = 0;
        public PointerJson(int pointer) { this.pointer = pointer; }
    }
}
