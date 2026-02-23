package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Constants;

public class PathRecorder {

    /**
     * One recorded sample — the motor powers at a single 20ms cycle.
     */
    public static class Sample {
        public final double leftPower;
        public final double rightPower;

        public Sample(double leftPower, double rightPower) {
            this.leftPower  = leftPower;
            this.rightPower = rightPower;
        }
    }

    // Multiple path slots
    private final List<List<Sample>> paths = new ArrayList<>();
    private int currentPathIndex = 0;

    private boolean recording = false;

    private boolean replaying = false;
    private int     replayIndex;

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

    /** Returns the current path slot's sample list. */
    private List<Sample> currentSamples() {
        return paths.get(currentPathIndex);
    }

    /** Cycles to the next path slot. */
    public void nextPath() {
        currentPathIndex = (currentPathIndex + 1) % Constants.NUM_PATHS;
    }

    /** Cycles to the previous path slot. */
    public void prevPath() {
        currentPathIndex = (currentPathIndex - 1 + Constants.NUM_PATHS) % Constants.NUM_PATHS;
    }

    /** Returns the current path slot index (0-based). */
    public int getCurrentPathIndex() {
        return currentPathIndex;
    }

    public void startRecording() {
        currentSamples().clear();
        recording = true;
    }

    public void stopRecording() {
        recording = false;
    }

    public boolean isRecording() {
        return recording;
    }

    /**
     * Records one cycle's motor powers. Called every 20ms while recording.
     */
    public void sample(double leftPower, double rightPower) {
        if (!recording) return;
        currentSamples().add(new Sample(leftPower, rightPower));
    }

    public void startReplay(double currentLeft, double currentRight) {
        if (currentSamples().isEmpty()) return;
        replaying = true;
        replayIndex = 0;
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
        return !currentSamples().isEmpty();
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

    /**
     * Returns the next sample's motor powers, or null if the replay is done.
     * Called once per 20ms cycle — plays back in real time.
     */
    public double[] replayStep(double currentLeft, double currentRight) {
        if (!replaying || replayIndex >= currentSamples().size()) {
            replaying = false;
            return null;
        }

        // Stall check: if the robot is trying to drive but the encoders
        // haven't changed, stop and return null.
        if (replayIndex > 0 && checkStall(currentLeft, currentRight)) {
            System.out.println("[PathRecorder] STALL DETECTED on path " + (currentPathIndex + 1)
                    + " at sample " + replayIndex + " - stopping replay.");
            replaying = false;
            return null;
        }

        Sample s = currentSamples().get(replayIndex);
        replayIndex++;

        return new double[] { s.leftPower, s.rightPower };
    }

    public int getSampleCount() {
        return currentSamples().size();
    }

    public int getReplayIndex() {
        return replayIndex;
    }
}
