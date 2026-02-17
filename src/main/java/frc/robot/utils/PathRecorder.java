package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

/**
 * Records and replays robot paths based on encoder distances.
 * 
 * During recording, each call to {@link #sample(double, double, double, double)}
 * captures the incremental left/right encoder distance traveled and the
 * motor power that was being applied. On replay, the segments are played
 * back at a fixed power so the path shape is preserved regardless of the
 * speed the driver originally drove.
 */
public class PathRecorder {

    /** One segment of a recorded path. */
    public static class Segment {
        /** Encoder distance the LEFT side must travel for this segment. */
        public final double leftDistance;
        /** Encoder distance the RIGHT side must travel for this segment. */
        public final double rightDistance;
        /** Motor output applied to the left side while recording. */
        public final double leftPower;
        /** Motor output applied to the right side while recording. */
        public final double rightPower;

        public Segment(double leftDistance, double rightDistance,
                       double leftPower, double rightPower) {
            this.leftDistance  = leftDistance;
            this.rightDistance = rightDistance;
            this.leftPower    = leftPower;
            this.rightPower   = rightPower;
        }
    }

    // ---- state ----
    private final List<Segment> segments = new ArrayList<>();
    private boolean recording = false;

    // snapshot of encoder positions at the previous sample
    private double prevLeft;
    private double prevRight;

    // replay state
    private boolean replaying = false;
    private int     replayIndex;
    private double  replayLeftStart;
    private double  replayRightStart;

    // ---- recording API --------------------------------------------------

    /**
     * Begin recording. Clears any previously recorded path.
     *
     * @param currentLeft  current left encoder distance
     * @param currentRight current right encoder distance
     */
    public void startRecording(double currentLeft, double currentRight) {
        segments.clear();
        prevLeft  = currentLeft;
        prevRight = currentRight;
        recording = true;
    }

    /** Stop recording and keep the captured segments. */
    public void stopRecording() {
        recording = false;
    }

    public boolean isRecording() {
        return recording;
    }

    /**
     * Call once per robot loop while recording.
     * Captures the distance traveled since the last sample and the motor
     * powers that produced that motion.
     *
     * @param currentLeft  current left encoder distance
     * @param currentRight current right encoder distance
     * @param leftPower    motor output currently applied to left side  (−1 … 1)
     * @param rightPower   motor output currently applied to right side (−1 … 1)
     */
    public void sample(double currentLeft, double currentRight,
                       double leftPower, double rightPower) {
        if (!recording) return;

        double dLeft  = currentLeft  - prevLeft;
        double dRight = currentRight - prevRight;

        // Only store if the robot actually moved or power was applied
        if (Math.abs(dLeft) > 0.01 || Math.abs(dRight) > 0.01 ||
            Math.abs(leftPower) > 0.01 || Math.abs(rightPower) > 0.01) {
            segments.add(new Segment(dLeft, dRight, leftPower, rightPower));
        }

        prevLeft  = currentLeft;
        prevRight = currentRight;
    }

    // ---- replay API -----------------------------------------------------

    /**
     * Begin replaying the recorded path.
     *
     * @param currentLeft  current left encoder distance
     * @param currentRight current right encoder distance
     */
    public void startReplay(double currentLeft, double currentRight) {
        if (segments.isEmpty()) return;
        replaying       = true;
        replayIndex     = 0;
        replayLeftStart  = currentLeft;
        replayRightStart = currentRight;
    }

    /** Cancel an in-progress replay. */
    public void stopReplay() {
        replaying = false;
    }

    public boolean isReplaying() {
        return replaying;
    }

    public boolean hasRecordedPath() {
        return !segments.isEmpty();
    }

    /**
     * Call once per robot loop during replay.
     * Returns the motor powers to apply, or {@code null} if the replay is
     * finished.
     *
     * @param currentLeft   current left encoder distance
     * @param currentRight  current right encoder distance
     * @param replaySpeed   constant speed multiplier (0 … 1) applied to output
     * @return double[2] {leftPower, rightPower}, or null when done
     */
    public double[] replayStep(double currentLeft, double currentRight,
                               double replaySpeed) {
        if (!replaying || replayIndex >= segments.size()) {
            replaying = false;
            return null;
        }

        Segment seg = segments.get(replayIndex);

        double leftTraveled  = currentLeft  - replayLeftStart;
        double rightTraveled = currentRight - replayRightStart;

        double targetLeft  = Math.abs(seg.leftDistance);
        double targetRight = Math.abs(seg.rightDistance);
        double target = Math.max(targetLeft, targetRight);

        // If this segment has essentially zero distance, skip it
        if (target < 0.05) {
            advanceSegment(currentLeft, currentRight);
            return replayStep(currentLeft, currentRight, replaySpeed);
        }

        boolean leftDone  = Math.abs(leftTraveled)  >= targetLeft;
        boolean rightDone = Math.abs(rightTraveled) >= targetRight;

        if (leftDone && rightDone) {
            advanceSegment(currentLeft, currentRight);
            return replayStep(currentLeft, currentRight, replaySpeed);
        }

        // Determine direction from recorded power signs
        double leftDir  = Math.signum(seg.leftPower);
        double rightDir = Math.signum(seg.rightPower);

        // Scale so the ratio between left and right is preserved
        double ratio;
        if (targetLeft > targetRight && targetRight > 0.001) {
            ratio = targetRight / targetLeft;
        } else if (targetRight > targetLeft && targetLeft > 0.001) {
            ratio = targetLeft / targetRight;
        } else {
            ratio = 1.0;
        }

        double leftOut, rightOut;
        if (targetLeft >= targetRight) {
            leftOut  = leftDir  * replaySpeed;
            rightOut = rightDir * replaySpeed * ratio;
        } else {
            leftOut  = leftDir  * replaySpeed * ratio;
            rightOut = rightDir * replaySpeed;
        }

        // Stop sides that have already reached their target
        if (leftDone)  leftOut  = 0;
        if (rightDone) rightOut = 0;

        return new double[] { leftOut, rightOut };
    }

    private void advanceSegment(double currentLeft, double currentRight) {
        replayIndex++;
        replayLeftStart  = currentLeft;
        replayRightStart = currentRight;
    }

    public int getSegmentCount() {
        return segments.size();
    }

    public int getReplayIndex() {
        return replayIndex;
    }
}
