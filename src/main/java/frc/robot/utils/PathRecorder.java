package frc.robot.utils;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

public class PathRecorder {

    public static class Sample {
        public final double leftPower;
        public final double rightPower;

        public Sample(double leftPower, double rightPower) {
            this.leftPower  = leftPower;
            this.rightPower = rightPower;
        }
    }

    private final List<List<Sample>> paths = new ArrayList<>();
    private int currentPathIndex = 0;

    private boolean recording = false;

    private boolean replaying = false;
    private int replayIndex;

    private double stallPrevLeft;
    private double stallPrevRight;
    private int    stallCounter;
    private static final double STALL_THRESHOLD = 0.5;

    private static final String PATHS_DIR = Filesystem.getOperatingDirectory().getAbsolutePath() + "/paths";

    public PathRecorder() {
        for (int i = 0; i < Constants.NUM_PATHS; i++) {
            paths.add(new ArrayList<>());
        }
        // Create the paths directory if it doesn't exist
        new File(PATHS_DIR).mkdirs();
        // Load any saved paths from disk
        loadAllPaths();
    }

    private List<Sample> currentSamples() {
        return paths.get(currentPathIndex);
    }

    public void nextPath() {
        currentPathIndex = (currentPathIndex + 1) % Constants.NUM_PATHS;
    }

    public void prevPath() {
        currentPathIndex = (currentPathIndex - 1 + Constants.NUM_PATHS) % Constants.NUM_PATHS;
    }

    public int getCurrentPathIndex() {
        return currentPathIndex;
    }

    public void startRecording() {
        currentSamples().clear();
        recording = true;
    }

    public void stopRecording() {
        recording = false;
        savePath(currentPathIndex);
    }

    public boolean isRecording() {
        return recording;
    }

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

    public double[] replayStep(double currentLeft, double currentRight) {
        if (!replaying || replayIndex >= currentSamples().size()) {
            replaying = false;
            return null;
        }

        Sample s = currentSamples().get(replayIndex);

        // Stall check: only check if the robot is actually being told to move.
        // If both powers are near zero, the robot is intentionally stationary.
        boolean commanding = Math.abs(s.leftPower) > 0.05 || Math.abs(s.rightPower) > 0.05;
        if (replayIndex > 0 && commanding && checkStall(currentLeft, currentRight)) {
            System.out.println("Stall detected on path " + (currentPathIndex + 1)
                    + " at sample " + replayIndex + ": stopping replay.");
            replaying = false;
            return null;
        }

        // Reset stall counter when not commanding movement
        if (!commanding) {
            stallCounter = 0;
            stallPrevLeft  = currentLeft;
            stallPrevRight = currentRight;
        }

        replayIndex++;

        return new double[] { s.leftPower, s.rightPower };
    }

    public int getSampleCount() {
        return currentSamples().size();
    }

    /** Returns the duration of the current path in seconds (each sample = 20ms). */
    public double getPathDuration() {
        return currentSamples().size() * 0.02;
    }

    public int getReplayIndex() {
        return replayIndex;
    }

    // ---- File I/O ----

    private String pathFile(int index) {
        return PATHS_DIR + "/path_" + (index + 1) + ".json";
    }

    /** Saves a single path slot to a JSON file. */
    public void savePath(int index) {
        List<Sample> samples = paths.get(index);
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(pathFile(index)))) {
            writer.write("[");
            for (int i = 0; i < samples.size(); i++) {
                Sample s = samples.get(i);
                if (i > 0) writer.write(",");
                writer.write("[" + s.leftPower + "," + s.rightPower + "]");
            }
            writer.write("]");
            System.out.println("[PathRecorder] Saved path " + (index + 1) + " to disk (" + samples.size() + " samples).");
        } catch (Exception e) {
            System.out.println("[PathRecorder] ERROR saving path " + (index + 1) + ": " + e.getMessage());
        }
    }

    /** Loads a single path slot from a JSON file, if it exists. */
    public void loadPath(int index) {
        File file = new File(pathFile(index));
        if (!file.exists()) return;

        List<Sample> samples = paths.get(index);
        samples.clear();

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                sb.append(line);
            }
            String json = sb.toString().trim();

            // Strip outer brackets
            if (json.startsWith("[")) json = json.substring(1);
            if (json.endsWith("]"))   json = json.substring(0, json.length() - 1);

            if (json.isEmpty()) return;

            // Parse each [left,right] pair
            int i = 0;
            while (i < json.length()) {
                int open = json.indexOf('[', i);
                if (open == -1) break;
                int close = json.indexOf(']', open);
                if (close == -1) break;

                String pair = json.substring(open + 1, close);
                String[] parts = pair.split(",");
                if (parts.length == 2) {
                    double left  = Double.parseDouble(parts[0].trim());
                    double right = Double.parseDouble(parts[1].trim());
                    samples.add(new Sample(left, right));
                }
                i = close + 1;
            }

            System.out.println("[PathRecorder] Loaded path " + (index + 1) + " from disk (" + samples.size() + " samples).");
        } catch (Exception e) {
            System.out.println("[PathRecorder] ERROR loading path " + (index + 1) + ": " + e.getMessage());
        }
    }

    /** Loads all path slots from disk. Called on startup. */
    private void loadAllPaths() {
        for (int i = 0; i < Constants.NUM_PATHS; i++) {
            loadPath(i);
        }
    }
}
