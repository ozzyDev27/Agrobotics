package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.PathRecorder;

public class DriveTrain extends SubsystemBase {
    
    //Initialize the motors
    private TitanQuad leftBack;
    private TitanQuad leftFront;
    private TitanQuad rightBack;
    private TitanQuad rightFront;

    public AHRS navX;

    private TitanQuadEncoder leftBackEncoder;
    private TitanQuadEncoder leftFrontEncoder;
    private TitanQuadEncoder rightBackEncoder;
    private TitanQuadEncoder rightFrontEncoder;

    // Path recording / replay
    private PathRecorder pathRecorder = new PathRecorder();
    // Track last motor outputs so the recorder can capture them
    private double lastLeftPower  = 0;
    private double lastRightPower = 0;

    public DriveTrain() { 
        leftBack = new TitanQuad(Constants.TITAN_ID, Constants.LEFT_BACK);
        leftFront = new TitanQuad(Constants.TITAN_ID, Constants.LEFT_FRONT);
        rightBack = new TitanQuad(Constants.TITAN_ID, Constants.RIGHT_BACK);
        rightFront = new TitanQuad(Constants.TITAN_ID, Constants.RIGHT_FRONT);

        Timer.delay(1);
                
        leftBackEncoder = new TitanQuadEncoder(leftBack, Constants.LEFT_BACK, Constants.DIST_PER_TICK);
        leftFrontEncoder = new TitanQuadEncoder(leftFront, Constants.LEFT_FRONT, Constants.DIST_PER_TICK);
        rightBackEncoder = new TitanQuadEncoder(rightBack, Constants.RIGHT_BACK, Constants.DIST_PER_TICK);
        rightFrontEncoder = new TitanQuadEncoder(rightFront, Constants.RIGHT_FRONT, Constants.DIST_PER_TICK);

        leftBackEncoder.setReverseDirection();
        leftFrontEncoder.setReverseDirection();
        
        rightBack.setInverted(true);
        rightFront.setInverted(true);
        
    }

    public void resetEncoders() {
        leftBackEncoder.reset();
        leftFrontEncoder.reset();
        rightBackEncoder.reset();
        rightFrontEncoder.reset();
    }

    public double getAverageEncoderDistance() {
        return (leftBackEncoder.getEncoderDistance() + leftFrontEncoder.getEncoderDistance() + rightBackEncoder.getEncoderDistance() + rightFrontEncoder.getEncoderDistance()) / 4.0;
    }

    public double leftBackEncoderDistance() {
        return leftBackEncoder.getEncoderDistance();
    }

    public double leftFrontEncoderDistance() {
        return leftFrontEncoder.getEncoderDistance();
    }

    public double rightBackEncoderDistance() {
        return rightBackEncoder.getEncoderDistance();
    }

    public double rightFrontEncoderDistance() {
        return rightFrontEncoder.getEncoderDistance();
    }

    // Simple arcade drive - x is turn, y is forward/backward
    public void driveArcade(double x, double y) {
        x = Math.max(-1.0, Math.min(1.0, x));
        y = Math.max(-1.0, Math.min(1.0, y));
        
        double left  = y + x;
        double right = y - x;

        // Left side: forward + turn
        leftBack.set(left);
        leftFront.set(left);
        
        // Right side: forward - turn
        rightBack.set(right);
        rightFront.set(right);

        // Track for path recorder
        lastLeftPower  = left;
        lastRightPower = right;
    }

    /**
     * Drive left and right sides independently (used during replay).
     */
    public void driveTank(double left, double right) {
        left  = Math.max(-1.0, Math.min(1.0, left));
        right = Math.max(-1.0, Math.min(1.0, right));

        leftBack.set(left);
        leftFront.set(left);
        rightBack.set(right);
        rightFront.set(right);

        lastLeftPower  = left;
        lastRightPower = right;
    }

    // ---- Encoder helpers for left/right averages ----

    public double getLeftEncoderDistance() {
        return (leftBackEncoder.getEncoderDistance()
              + leftFrontEncoder.getEncoderDistance()) / 2.0;
    }

    public double getRightEncoderDistance() {
        return (rightBackEncoder.getEncoderDistance()
              + rightFrontEncoder.getEncoderDistance()) / 2.0;
    }

    // ---- Path recording / replay ----

    /** Toggle recording on/off. Returns true if now recording, false if stopped. */
    public boolean toggleRecording() {
        if (pathRecorder.isRecording()) {
            pathRecorder.stopRecording();
            System.out.println("[PathRecorder] Stopped – " + pathRecorder.getSegmentCount() + " segments saved.");
            return false;
        } else {
            pathRecorder.startRecording(getLeftEncoderDistance(), getRightEncoderDistance());
            System.out.println("[PathRecorder] Recording started.");
            return true;
        }
    }

    /** Call every loop while the robot is being driven so segments are captured. */
    public void samplePath() {
        if (pathRecorder.isRecording()) {
            pathRecorder.sample(getLeftEncoderDistance(), getRightEncoderDistance(),
                                lastLeftPower, lastRightPower);
        }
    }

    public boolean isRecording() {
        return pathRecorder.isRecording();
    }

    /** Start replaying. Does nothing if no path has been recorded. */
    public void startReplay() {
        if (!pathRecorder.hasRecordedPath()) {
            System.out.println("[PathRecorder] Nothing recorded – cannot replay.");
            return;
        }
        pathRecorder.startReplay(getLeftEncoderDistance(), getRightEncoderDistance());
        System.out.println("[PathRecorder] Replay started (" + pathRecorder.getSegmentCount() + " segments).");
    }

    public void stopReplay() {
        pathRecorder.stopReplay();
        driveTank(0, 0);
    }

    public boolean isReplaying() {
        return pathRecorder.isReplaying();
    }

    /**
     * Advance one replay step. Returns true while still replaying, false when done.
     */
    public boolean replayStep(double replaySpeed) {
        double[] powers = pathRecorder.replayStep(
                getLeftEncoderDistance(), getRightEncoderDistance(), replaySpeed);
        if (powers == null) {
            driveTank(0, 0);
            System.out.println("[PathRecorder] Replay finished.");
            return false;
        }
        driveTank(powers[0], powers[1]);
        return true;
    }

    // Reset the gyro heading to zero
    public void resetGyro() {
        navX.zeroYaw();
    }

    @Override
    public void periodic() {
        // leftBack.set(3);
        // leftFront.set(3);
        // rightBack.set(0);
        // rightFront.set(0);
        // System.out.println("hi");
    }
}
