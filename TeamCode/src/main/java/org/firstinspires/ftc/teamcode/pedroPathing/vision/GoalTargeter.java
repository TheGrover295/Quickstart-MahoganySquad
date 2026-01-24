package org.firstinspires.ftc.teamcode.pedroPathing.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.Limelight;

/**
 * Uses the Limelight to track and target game elements.
 * Provides steering corrections and distance estimates for autonomous
 * alignment.
 * Implements full PID control for smooth tracking.
 */
public class GoalTargeter {

    private final Limelight limelight;

    // Tuning constants (Public for tuning)
    public static double STEERING_KP = 0.035;
    public static double STEERING_KI = 0.001;
    public static double STEERING_KD = 0.002;

    public static double DRIVE_KP = 0.025;
    public static double DRIVE_KI = 0.0; // Often not needed for drive, P+D is usually enough
    public static double DRIVE_KD = 0.005;

    public static double TARGET_AREA = 5.0; // Target area percentage for ideal distance
    public static double TX_TOLERANCE = 1.0; // Degrees of acceptable horizontal error
    public static double TA_TOLERANCE = 0.5; // Acceptable area error percentage

    // PID State
    private long lastTime = 0;
    private double steeringIntegral = 0;
    private double steeringLastError = 0;

    private double driveIntegral = 0;
    private double driveLastError = 0;

    private static final double INTEGRAL_MAX = 1.0; // Anti-windup cap

    // State
    private VisionData lastVisionData = VisionData.empty();
    private boolean isLocked = false;

    /**
     * Creates a GoalTargeter with the given Limelight.
     *
     * @param limelight The initialized Limelight subsystem.
     */
    public GoalTargeter(Limelight limelight) {
        this.limelight = limelight;
    }

    /**
     * Updates the targeter with the latest vision data.
     * Call this every loop iteration.
     */
    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            int pipeline = limelight.getCurrentPipeline();

            // Check for color pipeline results
            if (pipeline == Limelight.PIPELINE_GREEN || pipeline == Limelight.PIPELINE_PURPLE) {
                if (!limelight.getColorResults().isEmpty()) {
                    boolean isGreen = (pipeline == Limelight.PIPELINE_GREEN);
                    boolean isPurple = (pipeline == Limelight.PIPELINE_PURPLE);
                    lastVisionData = VisionData.fromColorTarget(
                            result.getTx(), result.getTy(), result.getTa(),
                            isGreen, isPurple);
                } else {
                    lastVisionData = VisionData.empty();
                }
            }
            // Check for AprilTag results
            else if (!result.getFiducialResults().isEmpty()) {
                int id = result.getFiducialResults().get(0).getFiducialId();
                lastVisionData = VisionData.withLocalization(
                        result.getTx(), result.getTy(), result.getTa(),
                        id, result.getBotpose(), result.getBotpose_MT2());
            }
            // Default: basic target data
            else {
                lastVisionData = VisionData.fromTarget(
                        result.getTx(),
                        result.getTy(),
                        result.getTa());
            }

            isLocked = isOnTarget();
        } else {
            lastVisionData = VisionData.empty();
            isLocked = false;
            resetPID();
        }
    }

    /**
     * Resets PID state variables.
     */
    private void resetPID() {
        steeringIntegral = 0;
        steeringLastError = 0;
        driveIntegral = 0;
        driveLastError = 0;
        lastTime = System.currentTimeMillis();
    }

    /**
     * Gets the steering correction needed to center on the target.
     * Positive = turn right, Negative = turn left.
     *
     * @return Steering power (-1.0 to 1.0), or 0 if no target.
     */
    public double getSteeringCorrection() {
        if (!lastVisionData.hasTarget()) {
            resetPID();
            return 0.0;
        }

        double error = -lastVisionData.getTx(); // Error is negative of tx (to turn towards 0)
        return calculatePID(error, STEERING_KP, STEERING_KI, STEERING_KD, true);
    }

    /**
     * Gets the drive correction needed to reach the target distance.
     * Positive = drive forward, Negative = drive backward.
     *
     * @return Drive power (-1.0 to 1.0), or 0 if no target.
     */
    public double getDriveCorrection() {
        if (!lastVisionData.hasTarget()) {
            resetPID();
            return 0.0;
        }

        // Area-based distance estimation
        double error = TARGET_AREA - lastVisionData.getTa();
        return calculatePID(error, DRIVE_KP, DRIVE_KI, DRIVE_KD, false);
    }

    /**
     * Generic PID calculation.
     */
    private double calculatePID(double error, double kP, double kI, double kD, boolean isSteering) {
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastTime) / 1000.0;

        // Handle first run or irregular timing
        if (dt <= 0.0001 || dt > 0.5) {
            dt = 0.02; // Default to ~50Hz
        }
        lastTime = currentTime;

        // Integral with anti-windup
        double integral = (isSteering ? steeringIntegral : driveIntegral) + (error * dt);
        if (Math.abs(integral) > INTEGRAL_MAX) {
            integral = Math.signum(integral) * INTEGRAL_MAX;
        }

        // Derivative
        double lastError = isSteering ? steeringLastError : driveLastError;
        double derivative = (error - lastError) / dt;

        // Store state
        if (isSteering) {
            steeringIntegral = integral;
            steeringLastError = error;
        } else {
            driveIntegral = integral;
            driveLastError = error;
        }

        double output = (error * kP) + (integral * kI) + (derivative * kD);
        return clamp(output, -1.0, 1.0);
    }

    /**
     * Checks if the robot is centered and at the correct distance.
     *
     * @return true if on target within tolerance.
     */
    public boolean isOnTarget() {
        if (!lastVisionData.hasTarget()) {
            return false;
        }

        boolean horizontallyCentered = Math.abs(lastVisionData.getTx()) < TX_TOLERANCE;
        boolean correctDistance = Math.abs(TARGET_AREA - lastVisionData.getTa()) < TA_TOLERANCE;

        return horizontallyCentered && correctDistance;
    }

    /**
     * Checks if a target is currently visible.
     *
     * @return true if target is detected.
     */
    public boolean hasTarget() {
        return lastVisionData.hasTarget() && lastVisionData.isFresh(500);
    }

    /**
     * Checks if the targeter has achieved lock on the target.
     *
     * @return true if locked on target.
     */
    public boolean isLocked() {
        return isLocked;
    }

    /**
     * Gets the latest vision data.
     *
     * @return The most recent VisionData.
     */
    public VisionData getVisionData() {
        return lastVisionData;
    }

    /**
     * Gets the horizontal offset to target.
     *
     * @return tx in degrees, or 0 if no target.
     */
    public double getTx() {
        return lastVisionData.getTx();
    }

    /**
     * Gets the vertical offset to target.
     *
     * @return ty in degrees, or 0 if no target.
     */
    public double getTy() {
        return lastVisionData.getTy();
    }

    /**
     * Gets the target area.
     *
     * @return ta as percentage, or 0 if no target.
     */
    public double getTa() {
        return lastVisionData.getTa();
    }

    /**
     * Switches the Limelight pipeline.
     *
     * @param pipeline Pipeline index (0-9).
     */
    public void setPipeline(int pipeline) {
        limelight.switchPipeline(pipeline);
    }

    /**
     * Clamps a value between min and max.
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Gets telemetry data as a formatted string.
     */
    public String getTelemetryString() {
        if (!lastVisionData.hasTarget()) {
            return "GoalTargeter: No Target";
        }
        return String.format("GoalTargeter: tx=%.1f° ty=%.1f° ta=%.1f%% locked=%s",
                lastVisionData.getTx(),
                lastVisionData.getTy(),
                lastVisionData.getTa(),
                isLocked ? "YES" : "NO");
    }
}