package org.firstinspires.ftc.teamcode.pedroPathing.AutoRecord;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.MecanumDrive;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Auto Replay Allen Test", group = "TeleOp")
public class AutoReplayAllenTest extends OpMode {

    Follower follower;
    MecanumDrive drivetrain;
    ReplayPID replayPID;
    Gamepad gamepadReplay1 = new Gamepad();
    Gamepad gamepadReplay2 = new Gamepad();

    private DcMotor leftFront, leftBack, rightFront, rightBack;

    PressHold recording;
    PressHold replay;
    PressHold pointerInput;
    Pose lastPose = new Pose(0, 0, 0);
    double lastTime = 0;
    double deltaTime = 0.02; // Record at 50Hz for smooth curves
    double deltaError = 2;
    int replayIndex = 0;

    private CubicSpline1D xSpline;
    private CubicSpline1D ySpline;
    private CubicSpline1D thetaSpline;

    boolean calculated;
    int currentGamepadIndex = 0;

    StateEntryJson currentReplayStates;
    List<TimePose> splineReplayStates;

    GamepadStateEntry lastGamePad1;
    GamepadStateEntry lastGamePad2;
    int logPointer = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        drivetrain = new MecanumDrive();
        drivetrain.init(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        recording = new PressHold(PressHold.PressType.DoublePress);
        replay = new PressHold(PressHold.PressType.DoublePress);
        pointerInput = new PressHold(PressHold.PressType.LongPress);
        replayPID = new ReplayPID(this);
        currentReplayStates = new StateEntryJson();

        loadPointer();
        telemetry.addData("pointer: ", logPointer);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        
        recording.checkStatus(gamepad1.a);
        replay.checkStatus(gamepad1.b);
        pointerInput.checkStatus(gamepad1.left_bumper);

        if (pointerInput.startPress) logPointer = 0;
        if (pointerInput.isOn) logPointer = (int) Math.floor(pointerInput.time.seconds());
        if (pointerInput.endPress) savePointer();

        if (recording.startPress) {
            recording.resetTimer();
            currentReplayStates = new StateEntryJson();
            lastPose = follower.getPose();
            if (lastPose == null) lastPose = new Pose(0,0,0);
            lastGamePad1 = new GamepadStateEntry(gamepad1);
            lastGamePad2 = new GamepadStateEntry(gamepad2);
            lastTime = 0;
        }
        
        if (recording.isOn) {
            Pose currentPose = follower.getPose();
            if (currentPose != null) {
                double elapsedTime = recording.time.seconds() - lastTime;
                if (elapsedTime > deltaTime) {
                    currentReplayStates.timeListPose.add(recording.time.seconds());
                    currentReplayStates.poseList.add(new PoseStateEntry(currentPose));
                    currentReplayStates.size += 1;
                    lastPose = currentPose;
                    lastTime = recording.time.seconds();
                }
            }
            if (!lastGamePad1.compareGamepad(new GamepadStateEntry(gamepad1)) || !lastGamePad2.compareGamepad(new GamepadStateEntry(gamepad2))) {
                lastGamePad1 = new GamepadStateEntry(gamepad1);
                lastGamePad2 = new GamepadStateEntry(gamepad2);
                currentReplayStates.timeListGamepad.add(recording.time.seconds());
                currentReplayStates.gamepad1List.add(new GamepadStateEntry(gamepad1));
                currentReplayStates.gamepad2List.add(new GamepadStateEntry(gamepad2));
            }
        }
        
        if (recording.endPress){
            recordPositions();
        }
        
        if (replay.startPress){
            loadPoses();
            buildCubicSpline();
            replay.resetTimer();
            currentGamepadIndex = 0;
            replayPID.reset();
        }
        
        if (replay.isOn){
            double currentTime = replay.time.seconds();
            Pose currentPose = follower.getPose();
            if (currentPose == null) currentPose = new Pose(0,0,0);
            
            double endTime = xSpline != null ? xSpline.endTime() : 0;
            double[] motorValues = replayPID.replayPIDMotorValues(currentTime, currentPose, endTime);
            if (motorValues == null) {
                replay.isOn = false;
                drivetrain.drive(0,0,0);
            } else {
                drivetrain.setMotorPowers(motorValues[0], motorValues[1], motorValues[2], motorValues[3]);
                
                if (currentGamepadIndex + 1 < currentReplayStates.timeListGamepad.size() &&
                        currentReplayStates.timeListGamepad.get(currentGamepadIndex + 1) <= currentTime) {
                    currentGamepadIndex++;
                }
                
                if (currentGamepadIndex < currentReplayStates.gamepad1List.size()) {
                    gamepadReplay1 = currentReplayStates.gamepad1List.get(currentGamepadIndex).convertToGamepad(1);
                    gamepadReplay2 = currentReplayStates.gamepad2List.get(currentGamepadIndex).convertToGamepad(2);
                }
            }
        }
        
        if (replay.endPress) {
            drivetrain.drive(0,0,0);
        }

        if (!replay.isOn) {
            double x = Math.pow(gamepad1.left_stick_x, 3);
            double y = -Math.pow(gamepad1.left_stick_y, 3);
            double rz = Math.pow(gamepad1.right_stick_x, 3);
            drivetrain.drive(y, x, rz);
        }
        
        telemetry.addData("Slot", logPointer);
        telemetry.addData("Recording", recording.isOn);
        telemetry.addData("Replaying", replay.isOn);
        telemetry.update();
    }

    public void buildCubicSpline(){
        splineReplayStates = new ArrayList<>();
        if (currentReplayStates == null || currentReplayStates.poseList == null || currentReplayStates.size == 0) return;
        
        // --- Heading Unwrapping ---
        double lastHeading = currentReplayStates.poseList.get(0).heading;
        double unwrappedHeading = lastHeading;

        for (int i = 0; i < currentReplayStates.size; i++){
            double currentHeading = currentReplayStates.poseList.get(i).heading;
            double deltaHeading = MathFunctions.normalizeAngle(currentHeading - lastHeading);
            unwrappedHeading += deltaHeading;
            lastHeading = currentHeading;

            splineReplayStates.add(new TimePose(currentReplayStates.poseList.get(i).x,
                    currentReplayStates.poseList.get(i).y,
                    currentReplayStates.timeListPose.get(i),
                    unwrappedHeading));
        }
        if (splineReplayStates.size() >= 2) {
            xSpline = new CubicSpline1D(splineReplayStates, pose -> pose.x);
            ySpline = new CubicSpline1D(splineReplayStates, pose -> pose.y);
            thetaSpline = new CubicSpline1D(splineReplayStates, pose -> pose.theta);
            calculated = true;
        }
    }

    public void recordPositions() {
        File dir = new File(AppUtil.ROOT_FOLDER + "/TeamCodeLogs");
        if (!dir.exists()) dir.mkdirs();
        File file = new File(dir, "movement" + logPointer + ".json");
        try (FileWriter writer = new FileWriter(file)) {
            Gson gson = new GsonBuilder().setPrettyPrinting().create();
            writer.write(gson.toJson(currentReplayStates));
        } catch (Exception e) {}
    }

    public void savePointer() {
        File dir = new File(AppUtil.ROOT_FOLDER + "/TeamCodeLogs");
        if (!dir.exists()) dir.mkdirs();
        File file = new File(dir, "pointer.json");
        try (FileWriter writer = new FileWriter(file)) {
            Gson gson = new GsonBuilder().create();
            writer.write(gson.toJson(new PointerJson(logPointer)));
        } catch (Exception e) {}
    }

    public void loadPoses() {
        File dir = new File(AppUtil.ROOT_FOLDER + "/TeamCodeLogs");
        File file = new File(dir, "movement" + logPointer + ".json");
        if (!file.exists()) return;
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            Gson gson = new GsonBuilder().create();
            currentReplayStates = gson.fromJson(reader, StateEntryJson.class);
        } catch (Exception e) {}
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

    public boolean IsReplayOn(){
        return replay != null && replay.isOn;
    }

    public Gamepad getGamepad1(){
        return gamepadReplay1;
    }
    public Gamepad getGamepad2(){
        return gamepadReplay2;
    }

    public double[] getTargets(double runtime){
        if (!calculated) {buildCubicSpline();}
        if (xSpline == null) return new double[9];
        double tNow = runtime;
        return new double[]{xSpline.evaluate(tNow), ySpline.evaluate(tNow), thetaSpline.evaluate(tNow), 
                            xSpline.derivative(tNow), ySpline.derivative(tNow), thetaSpline.derivative(tNow), 
                            xSpline.secondDerivative(tNow), ySpline.secondDerivative(tNow), thetaSpline.secondDerivative(tNow)};
    }

    public double[] getError(double runtime, Pose currentPose){
        if (!calculated) {buildCubicSpline();}
        if (xSpline == null) return new double[6];
        double tNow = runtime;
        return new double[]{xSpline.evaluate(tNow) - currentPose.getX(), 
                            ySpline.evaluate(tNow) - currentPose.getY(), 
                            MathFunctions.normalizeAngle(thetaSpline.evaluate(tNow) - currentPose.getHeading()),
                            xSpline.derivative(tNow) - follower.getVelocity().getXComponent(),
                            ySpline.derivative(tNow) - follower.getVelocity().getYComponent(),
                            MathFunctions.normalizeAngle(thetaSpline.derivative(tNow) - follower.getVelocity().getTheta())};
    }

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

        public Gamepad convertToGamepad(int gamepadNum) {
            Gamepad g = new Gamepad();
            if (gamepadNum == 2) g.a = this.a;
            g.b = this.b; g.x = this.x; g.y = this.y;
            g.dpad_up = this.dpad_up; g.dpad_down = this.dpad_down; g.dpad_left = this.dpad_left; g.dpad_right = this.dpad_right;
            g.left_bumper = this.left_bumper; g.right_bumper = this.right_bumper;
            g.left_stick_button = this.left_stick_button; g.right_stick_button = this.right_stick_button;
            g.left_stick_x = this.left_stick_x; g.left_stick_y = this.left_stick_y;
            g.right_stick_x = this.right_stick_x; g.right_stick_y = this.right_stick_y;
            g.left_trigger = this.left_trigger; g.right_trigger = this.right_trigger;
            return g;
        }

        public boolean compareGamepad(GamepadStateEntry that) {
            return a == that.a && b == that.b && x == that.x && y == that.y && dpad_up == that.dpad_up &&
                   dpad_down == that.dpad_down && dpad_left == that.dpad_left && dpad_right == that.dpad_right &&
                   left_bumper == that.left_bumper && right_bumper == that.right_bumper &&
                   left_stick_button == that.left_stick_button && right_stick_button == that.right_stick_button &&
                   Math.abs(left_trigger - that.left_trigger) < 0.001 && Math.abs(right_trigger - that.right_trigger) < 0.001;
        }
    }

    public static class PoseStateEntry {
        public double x, y, heading;
        public PoseStateEntry(Pose pose) { this.x = pose.getX(); this.y = pose.getY(); this.heading = pose.getHeading(); }
    }

    public static class StateEntryJson {
        public int size = 0;
        public List<Double> timeListPose = new ArrayList<>();
        public List<Double> timeListGamepad = new ArrayList<>();
        public List<PoseStateEntry> poseList = new ArrayList<>();
        public List<GamepadStateEntry> gamepad1List = new ArrayList<>();
        public List<GamepadStateEntry> gamepad2List = new ArrayList<>();
    }

    public static class PointerJson {
        public int pointer = 0;
        public PointerJson(int pointer){ this.pointer = pointer; }
    }
}
