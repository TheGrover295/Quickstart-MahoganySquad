package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Blue Auto Shoot", group = "AutoReal")
public class AutoShoot extends OpMode {

    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();
    private int pathState = 0;
    private int totalShotsFired = 0;

    // Hardware
    private DcMotor flywheelMotor, chamberSpinner;
    private Servo artifactTransfer;

    // Poses
    private final Pose startPose = new Pose(61.0, 6.7, Math.toRadians(270));
    private final Pose shootPose = new Pose(73.1, 29.6, Math.toRadians(298));
    private final Pose parkPose = new Pose(108.7, 10.4, Math.toRadians(100));

    // Paths
    private PathChain driveToShoot, driveToPark;

    // Constants & Updated Timings
    private final double TICKS_PER_STEP = 575.06; //475.06
    private final double SERVO_REST = 0.55;
    private final double SERVO_PUSH = 0.7;
    private double chamberTargetPos = 0;

    // Adjustable Timing Variables
    private final double INITIAL_WAIT = 5.0;  // 5 second wait at shoot pos
    private final double CHAMBER_WAIT = 6;  // Increased from 3s to 4s
    private final double ATM_RESET_WAIT = 2.0; // Increased from 1s to 2s

    public void buildPaths() {
        driveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveToPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");
        artifactTransfer = hardwareMap.get(Servo.class, "ATM");

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.6);

        artifactTransfer.setDirection(Servo.Direction.FORWARD);
        artifactTransfer.setPosition(SERVO_REST);

        buildPaths();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Drive to Shoot Position
                follower.followPath(driveToShoot);
                pathState = 1;
                break;

            case 1: // Wait for drive to finish, then start 5s initial wait
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 2;
                }
                break;

            case 2: // Initial 5 Second Wait
                if (timer.seconds() >= INITIAL_WAIT) {
                    pathState = 3; // Proceed to Shooting Loop
                }
                break;

            case 3: // Start Chamber Spin
                moveChamberStep();
                timer.reset();
                pathState = 4;
                break;

            case 4: // Wait for Chamber/Flywheel Spool (4 seconds)
                if (timer.seconds() >= CHAMBER_WAIT) {
                    flywheelMotor.setPower(1.0);
                    artifactTransfer.setDirection(Servo.Direction.REVERSE);
                    artifactTransfer.setPosition(SERVO_PUSH);
                    timer.reset();
                    pathState = 5;
                }
                break;

            case 5: // ATM back in and Wait for Reset (2 seconds)
                if (timer.seconds() >= 0.7) { // Hold push for 0.7s to ensure launch
                    artifactTransfer.setDirection(Servo.Direction.FORWARD);
                    artifactTransfer.setPosition(SERVO_REST);
                }

                if (timer.seconds() >= ATM_RESET_WAIT) {
                    totalShotsFired++;
                    if (totalShotsFired < 3) {
                        pathState = 3; // Loop back for next shot
                    } else {
                        flywheelMotor.setPower(0);
                        pathState = 6; // Go to Park
                    }
                }
                break;

            case 6: // Drive to Park Position
                follower.followPath(driveToPark);
                pathState = 7;
                break;

            case 7: // Final Check
                if (!follower.isBusy()) {
                    pathState = -1;
                }
                break;
        }
    }

    private void moveChamberStep() {
        chamberTargetPos += TICKS_PER_STEP;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setPower(0.6);
    }

    @Override
    public void start() {
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Shots Fired", totalShotsFired);
        telemetry.addData("Timer", timer.seconds());
        telemetry.update();
    }
}