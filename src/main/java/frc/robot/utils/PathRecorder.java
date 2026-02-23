package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Constants;

public class PathRecorder {

    public static class Segment {
        public final double leftDistance;
        public final double rightDistance;
        public final double leftPower;
        public final double rightPower;

        public Segment(double leftDistance, double rightDistance, double leftPower, double rightPower) {
            this.leftDistance  = leftDistance;
            this.rightDistance = rightDistance;
            this.leftPower    = leftPower;
            this.rightPower   = rightPower;
        }
    }

    // Multiple path slots
    private final List<List<Segment>> paths = new ArrayList<>();
    private int currentPathIndex = 0;

    private boolean recording = false;

    private double prevLeft;
    private double prevRight;

    private double accumLeft;
    private double accumRight;
    private double accumLeftPower;
    private double accumRightPower;
    private int    accumCount;

    private static final double MIN_SEGMENT_DIST = 50.0;

    private boolean replaying = false;
    private int     replayIndex;
    private double  replayLeftStart;
    private double  replayRightStart;

    // Stall detection
    private double stallPrevLeft;
    private double stallPrevRight;
    private int    stallCounter;
    private static final double STALL_THRESHOLD = 0.5;

    public PathRecorder() {
        for (int i = 0; i < Constants.NUM_PATHS; i++) {
            paths.add(new ArrayList<>());
        }
    }

    /** Returns the current path slot's segment list. */
    private List<Segment> currentSegments() {
        return paths.get(currentPathIndex);
    }

    /** Cycles to the next path slot. Cannot cycle while recording or replaying. */
    public void nextPath() {
        currentPathIndex = (currentPathIndex + 1) % Constants.NUM_PATHS;
    }

    /** Cycles to the previous path slot. Cannot cycle while recording or replaying. */
    public void prevPath() {
        currentPathIndex = (currentPathIndex - 1 + Constants.NUM_PATHS) % Constants.NUM_PATHS;
    }

    /** Returns the current path slot index (0-based). */
    public int getCurrentPathIndex() {
        return currentPathIndex;
    }

    public void startRecording(double currentLeft, double currentRight) {
        currentSegments().clear();
        prevLeft  = currentLeft;
        prevRight = currentRight;
        accumLeft = 0;
        accumRight = 0;
        accumLeftPower = 0;
        accumRightPower = 0;
        accumCount = 0;
        recording = true;
    }

    public void stopRecording() {
        if (recording) {
            flushAccumulator();
        }
        recording = false;
    }

    public boolean isRecording() {
        return recording;
    }

    public void sample(double currentLeft, double currentRight, double leftPower, double rightPower) {
        if (!recording) return;

        double dLeft  = currentLeft  - prevLeft;
        double dRight = currentRight - prevRight;

        prevLeft  = currentLeft;
        prevRight = currentRight;

        accumLeft  += dLeft;
        accumRight += dRight;

        if (Math.abs(leftPower) > 0.02 || Math.abs(rightPower) > 0.02) {
            accumLeftPower  += leftPower;
            accumRightPower += rightPower;
            accumCount++;
        }

        if (Math.max(Math.abs(accumLeft), Math.abs(accumRight)) >= MIN_SEGMENT_DIST) {
            commitSegment();
        }
    }

    private void flushAccumulator() {
        if (accumCount > 0 &&
            (Math.abs(accumLeft) > 0.01 || Math.abs(accumRight) > 0.01)) {
            commitSegment();
        }
    }

    private void commitSegment() {
        double avgLeftPower, avgRightPower;
        if (accumCount > 0) {
            avgLeftPower  = accumLeftPower  / accumCount;
            avgRightPower = accumRightPower / accumCount;
        } else {
            avgLeftPower  = (accumLeft  >= 0) ? 1.0 : -1.0;
            avgRightPower = (accumRight >= 0) ? 1.0 : -1.0;
        }
        currentSegments().add(new Segment(accumLeft, accumRight, avgLeftPower, avgRightPower));
        accumLeft  = 0;
        accumRight = 0;
        accumLeftPower  = 0;
        accumRightPower = 0;
        accumCount = 0;
    }

    public void startReplay(double currentLeft, double currentRight) {
        if (currentSegments().isEmpty()) return;
        replaying = true;
        replayIndex = 0;
        replayLeftStart = currentLeft;
        replayRightStart = currentRight;
        stallPrevLeft  = currentLeft;
        stallPrevRight = currentRight;
        stallCounter   = 0;
    }

    public void stopReplay() {
        replaying = false;
    }

    public boolean isReplaying() {
        return replaying;
    }

    public boolean hasRecordedPath() {
        return !currentSegments().isEmpty();
    }

    /**
     * Checks if the robot is stalled during replay.
     * Returns true if the encoders haven't moved for too many cycles.
     */
    private boolean checkStall(double currentLeft, double currentRight) {
        double leftDelta  = Math.abs(currentLeft  - stallPrevLeft);
        double rightDelta = Math.abs(currentRight - stallPrevRight);

        stallPrevLeft  = currentLeft;
        stallPrevRight = currentRight;

        if (leftDelta < STALL_THRESHOLD && rightDelta < STALL_THRESHOLD) {
            stallCounter++;
        } else {
            stallCounter = 0;
        }

        return stallCounter >= Constants.STALL_CHECK_CYCLES;
    }

    public double[] replayStep(double currentLeft, double currentRight,
                               double replaySpeed) {
        if (!replaying || replayIndex >= currentSegments().size()) {
            replaying = false;
            return null;
        }

        // Stall check: if the robot is trying to drive but the encoders
        // haven't changed, stop and return null.
        if (replayIndex > 0 && checkStall(currentLeft, currentRight)) {
            System.out.println("[PathRecorder] STALL DETECTED on path " + (currentPathIndex + 1) + " at segment " + replayIndex + " - stopping replay.");
            replaying = false;
            return null;
        }

        Segment seg = currentSegments().get(replayIndex);

        double leftTraveled  = currentLeft  - replayLeftStart;
        double rightTraveled = currentRight - replayRightStart;

        double targetLeft  = Math.abs(seg.leftDistance);
        double targetRight = Math.abs(seg.rightDistance);
        double target = Math.max(targetLeft, targetRight);

        // If the segment is tiny, advance to the next one but still return
        // a low-power output so we only skip one segment per call.
        if (target < 0.5) {
            advanceSegment(currentLeft, currentRight);
            return new double[] { 0.0, 0.0 };
        }

        double leftProgress  = (targetLeft  > 0.001) ? Math.abs(leftTraveled)  / targetLeft  : 1.0;
        double rightProgress = (targetRight > 0.001) ? Math.abs(rightTraveled) / targetRight : 1.0;
        double overallProgress = Math.max(leftProgress, rightProgress);

        if (overallProgress >= 0.98) {
            advanceSegment(currentLeft, currentRight);
            return new double[] { 0.0, 0.0 };
        }

        double leftDir  = Math.signum(seg.leftPower);
        double rightDir = Math.signum(seg.rightPower);

        double absLeft  = Math.abs(seg.leftPower);
        double absRight = Math.abs(seg.rightPower);
        double maxPow = Math.max(absLeft, absRight);

        double leftScale, rightScale;
        if (maxPow > 0.001) {
            leftScale  = absLeft  / maxPow;
            rightScale = absRight / maxPow;
        } else {
            leftScale  = 1.0;
            rightScale = 1.0;
        }

        double leftOut  = leftDir  * replaySpeed * leftScale;
        double rightOut = rightDir * replaySpeed * rightScale;

        double ramp = Math.min((1.0 - overallProgress) * 4.0, 1.0);
        ramp = Math.max(ramp, 0.1);
        leftOut  *= ramp;
        rightOut *= ramp;

        return new double[] { leftOut, rightOut };
    }

    private void advanceSegment(double currentLeft, double currentRight) {
        replayIndex++;
        replayLeftStart  = currentLeft;
        replayRightStart = currentRight;
    }

    public int getSegmentCount() {
        return currentSegments().size();
    }

    public int getReplayIndex() {
        return replayIndex;
    }
}
