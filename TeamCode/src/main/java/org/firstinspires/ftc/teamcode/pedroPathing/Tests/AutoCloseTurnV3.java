package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoRecord.PressHold;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.MecanumDrive;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "AutoCloseTurn V3", group = "Test")
public class AutoCloseTurnV3 extends OpMode {

    private Follower follower;
    private MecanumDrive drivetrain;

    private PressHold recording;
    private PressHold replay;
    private PressHold pointerInput;

    private StateEntryJson currentReplayStates;
    private int logPointer = 0;

    // recording
    private final double recordIntervalSec = 0.04;   // 25 Hz
    private double lastRecordTime = 0.0;

    // replay segment list
    private final List<ReplaySegment> replaySegments = new ArrayList<>();
    private int currentSegmentIndex = 0;
    private boolean replayActive = false;
    private boolean currentMoveStarted = false;

    // reduction / segmentation tuning
    private static final double MIN_POINT_SPACING = 1.25;                // inches
    private static final double MIN_HEADING_CHANGE_TO_KEEP = Math.toRadians(6);
    private static final double TURN_IN_PLACE_DIST_MAX = 1.5;            // inches
    private static final double TURN_IN_PLACE_HEADING_MIN = Math.toRadians(18);
    private static final double CONSTANT_HEADING_MAX_DELTA = Math.toRadians(6);
    private static final double COLLINEAR_BEND_MAX = Math.toRadians(5);

    // turn controller
    private static final double TURN_KP = 0.9;
    private static final double TURN_MIN = 0.16;
    private static final double TURN_MAX = 0.55;
    private static final double TURN_TOLERANCE = Math.toRadians(2.0);

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        drivetrain = new MecanumDrive();
        drivetrain.init(hardwareMap);

        recording = new PressHold(PressHold.PressType.DoublePress);
        replay = new PressHold(PressHold.PressType.DoublePress);
        pointerInput = new PressHold(PressHold.PressType.LongPress);

        currentReplayStates = new StateEntryJson();

        loadPointer();
        telemetry.addData("Pointer", logPointer);
        telemetry.update();
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

        // pointer slot select
        if (pointerInput.startPress) logPointer = 0;
        if (pointerInput.isOn) logPointer = (int) Math.floor(pointerInput.time.seconds());
        if (pointerInput.endPress) savePointer();

        // start recording
        if (recording.startPress) {
            currentReplayStates = new StateEntryJson();
            lastRecordTime = 0.0;

            Pose p = safePose(follower.getPose());
            currentReplayStates.poseList.add(new PoseStateEntry(p));
            currentReplayStates.size = currentReplayStates.poseList.size();
        }

        // record
        if (recording.isOn) {
            double t = recording.time.seconds();
            if (t - lastRecordTime >= recordIntervalSec) {
                Pose p = safePose(follower.getPose());
                currentReplayStates.poseList.add(new PoseStateEntry(p));
                currentReplayStates.size = currentReplayStates.poseList.size();
                lastRecordTime = t;
            }
        }

        // end recording
        if (recording.endPress) {
            recordPositions();
        }

        // start replay
        if (replay.startPress) {
            loadPoses();
            buildReplaySegments();
            currentSegmentIndex = 0;
            currentMoveStarted = false;
            replayActive = !replaySegments.isEmpty();

            if (replayActive) {
                Pose start = replaySegments.get(0).getStartPose();
                if (start != null) {
                    follower.setStartingPose(start);
                }
            }
        }

        // run replay state machine
        if (replay.isOn && replayActive) {
            runReplayStateMachine();
        }

        // stop replay
        if (replay.endPress) {
            stopReplay();
        }

        // manual drive only when not replaying
        if (!replay.isOn || !replayActive) {
            double x = Math.pow(gamepad1.left_stick_x, 3);
            double y = -Math.pow(gamepad1.left_stick_y, 3);
            double rz = Math.pow(gamepad1.right_stick_x, 3);
            drivetrain.drive(y, x, rz);
        }

        Pose pose = safePose(follower.getPose());

        telemetry.addData("Slot", logPointer);
        telemetry.addData("Recording", recording.isOn);
        telemetry.addData("Replay Button", replay.isOn);
        telemetry.addData("Replay Active", replayActive);
        telemetry.addData("Segment Index", currentSegmentIndex);
        telemetry.addData("Segment Count", replaySegments.size());
        telemetry.addData("Pose X", pose.getX());
        telemetry.addData("Pose Y", pose.getY());
        telemetry.addData("Pose H", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Recorded Poses", currentReplayStates.size);
        telemetry.update();
    }

    private void runReplayStateMachine() {
        if (currentSegmentIndex >= replaySegments.size()) {
            stopReplay();
            return;
        }

        ReplaySegment segment = replaySegments.get(currentSegmentIndex);

        if (segment.type == ReplaySegmentType.MOVE) {
            if (!currentMoveStarted) {
                follower.followPath(segment.pathChain);
                currentMoveStarted = true;
            }

            if (!follower.isBusy()) {
                currentSegmentIndex++;
                currentMoveStarted = false;
            }
            return;
        }

        if (segment.type == ReplaySegmentType.TURN) {
            Pose currentPose = safePose(follower.getPose());
            double error = MathFunctions.normalizeAngle(segment.targetHeading - currentPose.getHeading());

            if (Math.abs(error) <= TURN_TOLERANCE) {
                drivetrain.drive(0, 0, 0);
                currentSegmentIndex++;
                currentMoveStarted = false;
                return;
            }

            double turnCmd = TURN_KP * error;
            if (Math.abs(turnCmd) < TURN_MIN) {
                turnCmd = Math.signum(turnCmd) * TURN_MIN;
            }
            turnCmd = clamp(turnCmd, -TURN_MAX, TURN_MAX);

            drivetrain.drive(0, 0, turnCmd);
        }
    }

    private void stopReplay() {
        replayActive = false;
        currentMoveStarted = false;
        currentSegmentIndex = 0;
        drivetrain.drive(0, 0, 0);
    }

    private void buildReplaySegments() {
        replaySegments.clear();

        if (currentReplayStates == null || currentReplayStates.poseList == null || currentReplayStates.poseList.size() < 2) {
            return;
        }

        List<Pose> raw = new ArrayList<>();
        for (PoseStateEntry entry : currentReplayStates.poseList) {
            raw.add(entry.toPose());
        }

        List<Pose> reduced = reduceRecordedPoses(raw);
        if (reduced.size() < 2) return;

        List<Pose> moveBuffer = new ArrayList<>();
        moveBuffer.add(reduced.get(0));

        for (int i = 1; i < reduced.size(); i++) {
            Pose prev = reduced.get(i - 1);
            Pose curr = reduced.get(i);

            if (isTurnInPlace(prev, curr)) {
                // flush any accumulated move segment first
                if (moveBuffer.size() >= 2) {
                    PathChain movePath = buildMovePath(moveBuffer);
                    if (movePath != null) {
                        replaySegments.add(ReplaySegment.move(movePath, moveBuffer.get(0), moveBuffer.get(moveBuffer.size() - 1)));
                    }
                }

                moveBuffer.clear();
                moveBuffer.add(curr);

                replaySegments.add(ReplaySegment.turn(prev, curr.getHeading()));
            } else {
                moveBuffer.add(curr);
            }
        }

        if (moveBuffer.size() >= 2) {
            PathChain movePath = buildMovePath(moveBuffer);
            if (movePath != null) {
                replaySegments.add(ReplaySegment.move(movePath, moveBuffer.get(0), moveBuffer.get(moveBuffer.size() - 1)));
            }
        }
    }

    private PathChain buildMovePath(List<Pose> poses) {
        if (poses == null || poses.size() < 2) return null;

        PathBuilder builder = follower.pathBuilder();

        for (int i = 0; i < poses.size() - 1; i++) {
            Pose p0 = poses.get(i);
            Pose p1 = poses.get(i + 1);

            builder.addPath(new BezierLine(p0, p1));

            double headingDelta = Math.abs(MathFunctions.normalizeAngle(p1.getHeading() - p0.getHeading()));

            if (headingDelta <= CONSTANT_HEADING_MAX_DELTA) {
                builder.setConstantHeadingInterpolation(p0.getHeading());
            } else {
                builder.setLinearHeadingInterpolation(p0.getHeading(), p1.getHeading());
            }
        }

        return builder.build();
    }

    private List<Pose> reduceRecordedPoses(List<Pose> raw) {
        List<Pose> reduced = new ArrayList<>();
        if (raw.isEmpty()) return reduced;

        Pose lastKept = raw.get(0);
        reduced.add(lastKept);

        for (int i = 1; i < raw.size() - 1; i++) {
            Pose current = raw.get(i);

            double dist = distance(lastKept, current);
            double headingDelta = Math.abs(MathFunctions.normalizeAngle(current.getHeading() - lastKept.getHeading()));

            if (dist >= MIN_POINT_SPACING || headingDelta >= MIN_HEADING_CHANGE_TO_KEEP) {
                reduced.add(current);
                lastKept = current;
            }
        }

        Pose last = raw.get(raw.size() - 1);
        if (distance(reduced.get(reduced.size() - 1), last) > 0.05 ||
                Math.abs(MathFunctions.normalizeAngle(last.getHeading() - reduced.get(reduced.size() - 1).getHeading())) > Math.toRadians(1)) {
            reduced.add(last);
        }

        return removeCollinearInteriorPoints(reduced);
    }

    private List<Pose> removeCollinearInteriorPoints(List<Pose> points) {
        if (points.size() <= 2) return points;

        List<Pose> result = new ArrayList<>();
        result.add(points.get(0));

        for (int i = 1; i < points.size() - 1; i++) {
            Pose a = points.get(i - 1);
            Pose b = points.get(i);
            Pose c = points.get(i + 1);

            double angleAB = Math.atan2(b.getY() - a.getY(), b.getX() - a.getX());
            double angleBC = Math.atan2(c.getY() - b.getY(), c.getX() - b.getX());
            double bend = Math.abs(MathFunctions.normalizeAngle(angleBC - angleAB));

            double headingAB = Math.abs(MathFunctions.normalizeAngle(b.getHeading() - a.getHeading()));
            double headingBC = Math.abs(MathFunctions.normalizeAngle(c.getHeading() - b.getHeading()));

            boolean keep = bend > COLLINEAR_BEND_MAX
                    || headingAB > MIN_HEADING_CHANGE_TO_KEEP
                    || headingBC > MIN_HEADING_CHANGE_TO_KEEP;

            if (keep) {
                result.add(b);
            }
        }

        result.add(points.get(points.size() - 1));
        return result;
    }

    private boolean isTurnInPlace(Pose a, Pose b) {
        double dist = distance(a, b);
        double headingDelta = Math.abs(MathFunctions.normalizeAngle(b.getHeading() - a.getHeading()));

        return dist <= TURN_IN_PLACE_DIST_MAX && headingDelta >= TURN_IN_PLACE_HEADING_MIN;
    }

    private double distance(Pose a, Pose b) {
        return Math.hypot(b.getX() - a.getX(), b.getY() - a.getY());
    }

    private Pose safePose(Pose pose) {
        return pose != null ? pose : new Pose(0, 0, 0);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    public void recordPositions() {
        File dir = new File(AppUtil.ROOT_FOLDER + "/TeamCodeLogs");
        if (!dir.exists()) dir.mkdirs();

        File file = new File(dir, "movement" + logPointer + ".json");
        try (FileWriter writer = new FileWriter(file)) {
            Gson gson = new GsonBuilder().setPrettyPrinting().create();
            writer.write(gson.toJson(currentReplayStates));
        } catch (Exception e) {
            telemetry.addData("Save Error", e.getMessage());
        }
    }

    public void savePointer() {
        File dir = new File(AppUtil.ROOT_FOLDER + "/TeamCodeLogs");
        if (!dir.exists()) dir.mkdirs();

        File file = new File(dir, "pointer.json");
        try (FileWriter writer = new FileWriter(file)) {
            Gson gson = new GsonBuilder().create();
            writer.write(gson.toJson(new PointerJson(logPointer)));
        } catch (Exception e) {
            telemetry.addData("Pointer Save Error", e.getMessage());
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
            telemetry.addData("Load Error", e.getMessage());
        }
    }

    public void loadPointer() {
        File dir = new File(AppUtil.ROOT_FOLDER + "/TeamCodeLogs");
        File file = new File(dir, "pointer.json");
        if (!file.exists()) return;

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            Gson gson = new GsonBuilder().create();
            PointerJson pointerJson = gson.fromJson(reader, PointerJson.class);
            if (pointerJson != null) {
                logPointer = pointerJson.pointer;
            }
        } catch (Exception e) {
            telemetry.addData("Pointer Load Error", e.getMessage());
        }
    }

    private enum ReplaySegmentType {
        MOVE,
        TURN
    }

    private static class ReplaySegment {
        ReplaySegmentType type;
        PathChain pathChain;
        double targetHeading;
        Pose startPose;
        Pose endPose;

        static ReplaySegment move(PathChain pathChain, Pose startPose, Pose endPose) {
            ReplaySegment s = new ReplaySegment();
            s.type = ReplaySegmentType.MOVE;
            s.pathChain = pathChain;
            s.startPose = startPose;
            s.endPose = endPose;
            return s;
        }

        static ReplaySegment turn(Pose startPose, double targetHeading) {
            ReplaySegment s = new ReplaySegment();
            s.type = ReplaySegmentType.TURN;
            s.startPose = startPose;
            s.endPose = new Pose(startPose.getX(), startPose.getY(), targetHeading);
            s.targetHeading = targetHeading;
            return s;
        }

        Pose getStartPose() {
            return startPose;
        }
    }

    public static class PoseStateEntry {
        public double x, y, heading;

        public PoseStateEntry() { }

        public PoseStateEntry(Pose pose) {
            this.x = pose.getX();
            this.y = pose.getY();
            this.heading = pose.getHeading();
        }

        public Pose toPose() {
            return new Pose(x, y, heading);
        }
    }

    public static class StateEntryJson {
        public int size = 0;
        public List<PoseStateEntry> poseList = new ArrayList<>();
    }

    public static class PointerJson {
        public int pointer = 0;

        public PointerJson(int pointer) {
            this.pointer = pointer;
        }
    }
}
