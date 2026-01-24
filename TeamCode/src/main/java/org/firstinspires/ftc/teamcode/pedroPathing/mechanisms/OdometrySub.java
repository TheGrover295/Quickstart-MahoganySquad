package org.firstinspires.ftc.teamcode.pedroPathing.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Updated Odometry Subsystem to use GoBilda Pinpoint via Pedro Pathing.
 */
public class OdometrySub {

    private Follower follower;

    /**
     * Initializes the Pedro Pathing follower which handles all Pinpoint logic.
     * @param hwMap The hardware map from the OpMode.
     */
    public void init(HardwareMap hwMap) {
        // We use the builder you defined in your Constants file
        follower = Constants.createFollower(hwMap);
    }

    /**
     * Updates the follower. This must be called every loop.
     * This updates the Pinpoint and the Pathing engines.
     */
    public void update() {
        follower.update();
    }

    /**
     * Resets the pose to (0,0,0).
     */
    public void reset() {
        follower.setPose(new Pose(0, 0, 0));
    }

    /**
     * Sets the current position (Field Centric).
     */
    public void setPosition(double x, double y, double headingRadians) {
        follower.setPose(new Pose(x, y, headingRadians));
    }

    // --- Position Getters (Pulling from the Pinpoint via Follower) ---

    public double getX() {
        return follower.getPose().getX();
    }

    public double getY() {
        return follower.getPose().getY();
    }

    public double getHeading() {
        return follower.getPose().getHeading();
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(getHeading());
    }

    /**
     * Returns telemetry string for debugging.
     */
    public String getTelemetryString() {
        Pose currentPose = follower.getPose();
        return String.format("Pinpoint Pos: (%.1f, %.1f) in | Heading: %.1f°",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
    }

    public Follower getFollower() {
        return follower;
    }
}