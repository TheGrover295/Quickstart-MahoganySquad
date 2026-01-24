package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Limelight Smart Auto", group = "AutoReal")
@Configurable
public class LimelightMotif extends LinearOpMode {
    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Initialize poses
    private final Pose startPose = new Pose(72, 120, Math.toRadians(90));
    private final Pose scorePose = new Pose(72, 20, Math.toRadians(115));
    private final Pose PPGPose = new Pose(100, 83.5, Math.toRadians(0));
    private final Pose PGPPose = new Pose(100, 59.5, Math.toRadians(0));
    private final Pose GPPPose = new Pose(100, 35.5, Math.toRadians(0));

    // Path Chains
    private PathChain grabPPG, scorePPG;
    private PathChain grabPGP, scorePGP;
    private PathChain grabGPP, scoreGPP;

    // April Tag IDs
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;

    // LIMELIGHT HARDWARE
    private Limelight3A limelight;

    // Other variables
    private Pose currentPose;
    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private int pathStatePPG, pathStatePGP, pathStateGPP;
    private int foundID = -1; // -1 means no tag pattern locked in yet

    // Mechanisms
    private DcMotor flywheelMotor;
    private DcMotor chamberSpinner;
    private CRServo artifactTransfer;
    private DcMotor intakeMotor;

    // Constants
    private double chamberTargetPos = 0;
    private final double TICKS_PER_STEP = 490;
    private final double INTAKE_POWER = 1.0;
    private final double FLYWHEEL_POWER = 0.64;

    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
            panelsTelemetry.debug(caption + ": " + text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panelsTelemetry.debug(caption + ": " + message);
        }
    }

    private void safeWait(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            follower.update();
            panelsTelemetry.update();
        }
    }

    public void intakeArtifacts() {
        intakeMotor.setPower(INTAKE_POWER);
        safeWait(1.5);
        intakeMotor.setPower(0);
    }

    public void shootArtifacts() {
        flywheelMotor.setPower(FLYWHEEL_POWER);
        chamberTargetPos += TICKS_PER_STEP;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setPower(0.72);
        safeWait(1.8);
        artifactTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        artifactTransfer.setPower(1);
        safeWait(0.9);
        artifactTransfer.setPower(0);
        flywheelMotor.setPower(0);
    }

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Hardware Init
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");
        artifactTransfer = hardwareMap.get(CRServo.class, "ATM");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.63);

        // --- LIMELIGHT INIT ---
        // We get it as a Limelight3A class, NOT a WebcamName
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Force the Limelight to Pipeline 0 (Standard AprilTags)
        limelight.pipelineSwitch(0);

        // Begin streaming data
        limelight.start();

        log("Status", "Initialized - Waiting for Start");
        telemetry.update();

        waitForStart();
        runtime.reset();

        setpathStatePPG(0);
        setpathStatePGP(0);
        setpathStateGPP(0);

        // Main Loop
        while (opModeIsActive()) {
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose();

            // --- READ LIMELIGHT DATA ---
            // Only look for tags if we haven't locked onto a path yet (foundID == -1)
            if (foundID == -1) {
                LLResult result = limelight.getLatestResult();

                // Check if result is valid and has tags
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                    for (LLResultTypes.FiducialResult tag : fiducials) {
                        int id = tag.getFiducialId();

                        if (id == PPG_TAG_ID) {
                            buildPathsPPG();
                            foundID = 21; // Set the ID to lock this path
                            log("Path Found", "PPG (ID 23)");
                            break;
                        } else if (id == PGP_TAG_ID) {
                            buildPathsPGP();
                            foundID = 22;
                            log("Path Found", "PGP (ID 22)");
                            break;
                        } else if (id == GPP_TAG_ID) {
                            buildPathsGPP();
                            foundID = 23;
                            log("Path Found", "GPP (ID 21)");
                            break;
                        }
                    }
                }
            }

            // --- STATE MACHINES ---
            if (foundID == 21) {
                updateStateMachinePPG();
            } else if (foundID == 22) {
                updateStateMachinePGP();
            } else if (foundID == 23) {
                updateStateMachineGPP();
            }

            log("Elapsed", runtime.toString());
            log("State", foundID == -1 ? "Scanning..." : "Running " + foundID);
            telemetry.update();
        }

        // Stop the Limelight when OpMode ends
        limelight.stop();
    }

    // [Path Building and State Machine methods remain identical to your previous code]

    public void buildPathsPPG() {
        grabPPG = follower.pathBuilder()
                .addPath(new BezierLine(startPose, PPGPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), PPGPose.getHeading())
                .build();
        scorePPG = follower.pathBuilder()
                .addPath(new BezierLine(PPGPose, scorePose))
                .setLinearHeadingInterpolation(PPGPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void buildPathsPGP() {
        grabPGP = follower.pathBuilder()
                .addPath(new BezierLine(startPose, PGPPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), PGPPose.getHeading())
                .build();
        scorePGP = follower.pathBuilder()
                .addPath(new BezierLine(PGPPose, scorePose))
                .setLinearHeadingInterpolation(PGPPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void buildPathsGPP() {
        grabGPP = follower.pathBuilder()
                .addPath(new BezierLine(startPose, GPPPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), GPPPose.getHeading())
                .build();
        scoreGPP = follower.pathBuilder()
                .addPath(new BezierLine(GPPPose, scorePose))
                .setLinearHeadingInterpolation(GPPPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void updateStateMachinePPG() {
        switch (pathStatePPG) {
            case 0:
                follower.followPath(grabPPG);
                setpathStatePPG(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    intakeArtifacts();
                    follower.followPath(scorePPG);
                    setpathStatePPG(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    shootArtifacts();
                    setpathStatePPG(-1);
                }
                break;
        }
    }

    public void updateStateMachinePGP() {
        switch (pathStatePGP) {
            case 0:
                follower.followPath(grabPGP);
                setpathStatePGP(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    intakeArtifacts();
                    follower.followPath(scorePGP);
                    setpathStatePGP(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    shootArtifacts();
                    setpathStatePGP(-1);
                }
                break;
        }
    }

    public void updateStateMachineGPP() {
        switch (pathStateGPP) {
            case 0:
                follower.followPath(grabGPP);
                setpathStateGPP(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    intakeArtifacts();
                    follower.followPath(scoreGPP);
                    setpathStateGPP(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    shootArtifacts();
                    setpathStateGPP(-1);
                }
                break;
        }
    }

    void setpathStatePPG(int newPathState) { this.pathStatePPG = newPathState; }
    void setpathStatePGP(int newPathState) { this.pathStatePGP = newPathState; }
    void setpathStateGPP(int newPathState) { this.pathStateGPP = newPathState; }
}