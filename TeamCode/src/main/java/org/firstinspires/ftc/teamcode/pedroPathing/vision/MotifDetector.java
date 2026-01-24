package org.firstinspires.ftc.teamcode.pedroPathing.vision;

/**
 * Detects and interprets the MOTIF from the OBELISK AprilTags.
 * OBELISK tags: ID 21 (GPP), ID 22 (PGP), ID 23 (PPG)
 *
 * Supports both direct Limelight API and VisionData-based API.
 */
public class MotifDetector {

    // OBELISK AprilTag IDs
    public static final int OBELISK_TAG_GPP = 21; // Green, Purple, Purple
    public static final int OBELISK_TAG_PGP = 22; // Purple, Green, Purple
    public static final int OBELISK_TAG_PPG = 23; // Purple, Purple, Green

    /**
     * Enum representing the three possible MOTIF patterns.
     */
    public enum Motif {
        GPP("GPP", new char[] { 'G', 'P', 'P' }),
        PGP("PGP", new char[] { 'P', 'G', 'P' }),
        PPG("PPG", new char[] { 'P', 'P', 'G' }),
        UNKNOWN("UNKNOWN", new char[] {});

        private final String name;
        private final char[] pattern;

        Motif(String name, char[] pattern) {
            this.name = name;
            this.pattern = pattern;
        }

        /**
         * Gets the color at a specific index (0-8) with pattern repetition.
         */
        public char getColorAtIndex(int index) {
            if (pattern.length == 0)
                return '?';
            return pattern[index % 3];
        }

        /**
         * Gets the full 9-position ramp pattern.
         */
        public char[] getFullRampPattern() {
            if (pattern.length == 0)
                return new char[0];
            char[] full = new char[9];
            for (int i = 0; i < 9; i++) {
                full[i] = pattern[i % 3];
            }
            return full;
        }

        @Override
        public String toString() {
            return name;
        }
    }

    // State tracking
    private Motif detectedMotif = Motif.UNKNOWN;
    private int consecutiveDetections = 0;
    private int lastDetectedTagId = -1;
    private long lastDetectionTime = 0;

    // Configuration
    private static final int CONFIDENCE_THRESHOLD = 3; // Require 3 consecutive detections
    private static final long TIMEOUT_MS = 1000; // Reset after 1 second of no detections

    /**
     * Updates the detector with new vision data.
     * Call this every loop iteration.
     *
     * @param visionData The latest vision data from Limelight.
     */
    public void update(VisionData visionData) {
        if (visionData == null || !visionData.hasTarget()) {
            // Check for timeout
            if (System.currentTimeMillis() - lastDetectionTime > TIMEOUT_MS) {
                consecutiveDetections = 0;
                lastDetectedTagId = -1;
            }
            return;
        }

        int tagId = visionData.getTagID();
        Motif currentMotif = tagIdToMotif(tagId);

        if (currentMotif != Motif.UNKNOWN) {
            lastDetectionTime = System.currentTimeMillis();

            if (tagId == lastDetectedTagId) {
                consecutiveDetections++;
            } else {
                lastDetectedTagId = tagId;
                consecutiveDetections = 1;
            }

            if (consecutiveDetections >= CONFIDENCE_THRESHOLD) {
                detectedMotif = currentMotif;
            }
        }
    }

    /**
     * Checks if we have a confident detection (3+ consecutive frames).
     *
     * @return true if confident detection exists.
     */
    public boolean hasConfidentDetection() {
        return detectedMotif != Motif.UNKNOWN && consecutiveDetections >= CONFIDENCE_THRESHOLD;
    }

    /**
     * Gets the detected motif.
     *
     * @return The detected Motif enum, or Motif.UNKNOWN if not detected.
     */
    public Motif getDetectedMotif() {
        return detectedMotif;
    }

    /**
     * Converts an AprilTag ID to the corresponding Motif.
     */
    private Motif tagIdToMotif(int tagId) {
        switch (tagId) {
            case OBELISK_TAG_GPP:
                return Motif.GPP;
            case OBELISK_TAG_PGP:
                return Motif.PGP;
            case OBELISK_TAG_PPG:
                return Motif.PPG;
            default:
                return Motif.UNKNOWN;
        }
    }

    /**
     * Resets the detector state.
     */
    public void reset() {
        detectedMotif = Motif.UNKNOWN;
        consecutiveDetections = 0;
        lastDetectedTagId = -1;
        lastDetectionTime = 0;
    }

    /**
     * Gets a human-readable pattern description.
     */
    public String getPatternDescription() {
        if (detectedMotif == Motif.UNKNOWN) {
            return "No pattern detected";
        }
        char[] pattern = detectedMotif.getFullRampPattern();
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < pattern.length; i++) {
            if (i > 0 && i % 3 == 0)
                sb.append(" | ");
            sb.append(pattern[i]);
        }
        return sb.toString();
    }

    /**
     * Gets the number of consecutive detections.
     */
    public int getConsecutiveDetections() {
        return consecutiveDetections;
    }
}