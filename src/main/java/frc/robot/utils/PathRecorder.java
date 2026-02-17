package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

public class PathRecorder {

    public static class Segment {
        public final double leftDistance;
        public final double rightDistance;
        public final double leftPower;
        public final double rightPower;

        public Segment(double leftDistance, double rightDistance,
                       double leftPower, double rightPower) {
            this.leftDistance  = leftDistance;
            this.rightDistance = rightDistance;
            this.leftPower    = leftPower;
            this.rightPower   = rightPower;
        }
    }

    private final List<Segment> segments = new ArrayList<>();
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

    public void startRecording(double currentLeft, double currentRight) {
        segments.clear();
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

    public void sample(double currentLeft, double currentRight,
                       double leftPower, double rightPower) {
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
        segments.add(new Segment(accumLeft, accumRight, avgLeftPower, avgRightPower));
        accumLeft  = 0;
        accumRight = 0;
        accumLeftPower  = 0;
        accumRightPower = 0;
        accumCount = 0;
    }

    public void startReplay(double currentLeft, double currentRight) {
        if (segments.isEmpty()) return;
        replaying        = true;
        replayIndex      = 0;
        replayLeftStart  = currentLeft;
        replayRightStart = currentRight;
    }

    public void stopReplay() {
        replaying = false;
    }

    public boolean isReplaying() {
        return replaying;
    }

    public boolean hasRecordedPath() {
        return !segments.isEmpty();
    }

    public double[] replayStep(double currentLeft, double currentRight,
                               double replaySpeed) {
        while (replaying && replayIndex < segments.size()) {
            Segment seg = segments.get(replayIndex);

            double leftTraveled  = currentLeft  - replayLeftStart;
            double rightTraveled = currentRight - replayRightStart;

            double targetLeft  = Math.abs(seg.leftDistance);
            double targetRight = Math.abs(seg.rightDistance);
            double target = Math.max(targetLeft, targetRight);

            if (target < 0.5) {
                advanceSegment(currentLeft, currentRight);
                continue;
            }

            double leftProgress  = (targetLeft  > 0.001) ? Math.abs(leftTraveled)  / targetLeft  : 1.0;
            double rightProgress = (targetRight > 0.001) ? Math.abs(rightTraveled) / targetRight : 1.0;

            if (leftProgress >= 1.0 && rightProgress >= 1.0) {
                advanceSegment(currentLeft, currentRight);
                continue;
            }

            double leftDir  = Math.signum(seg.leftPower);
            double rightDir = Math.signum(seg.rightPower);

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

            double minPower = 0.15;

            if (leftProgress >= 1.0) {
                leftOut = 0;
            } else {
                double remaining = 1.0 - leftProgress;
                double ramp = Math.min(remaining * 3.0, 1.0);
                leftOut = leftOut * Math.max(ramp, minPower / Math.max(Math.abs(leftOut), 0.001));
            }

            if (rightProgress >= 1.0) {
                rightOut = 0;
            } else {
                double remaining = 1.0 - rightProgress;
                double ramp = Math.min(remaining * 3.0, 1.0);
                rightOut = rightOut * Math.max(ramp, minPower / Math.max(Math.abs(rightOut), 0.001));
            }

            return new double[] { leftOut, rightOut };
        }

        replaying = false;
        return null;
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
