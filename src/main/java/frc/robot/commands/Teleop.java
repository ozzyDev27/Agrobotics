package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.gamepad.OI;
import frc.robot.subsystems.DriveTrain;

public class Teleop extends CommandBase {
    
    private static final DriveTrain driveTrain = RobotContainer.driveTrain;
    private static final OI oi = RobotContainer.oi;

    public Teleop() {
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        // Reset gyro when teleop starts
        driveTrain.resetGyro();
    }

    @Override
    public void execute() {

        // --- X button: toggle path recording on / off ---
        if (oi.getDriveXButton()) {
            // If we're replaying, cancel replay first
            if (driveTrain.isReplaying()) {
                driveTrain.stopReplay();
            }
            driveTrain.toggleRecording();
        }

        // --- Y button: start / stop replay ---
        if (oi.getDriveYButton()) {
            if (driveTrain.isReplaying()) {
                driveTrain.stopReplay();
            } else if (!driveTrain.isRecording()) {
                driveTrain.startReplay();
            }
        }

        // --- Drive or replay ---
        if (driveTrain.isReplaying()) {
            // Let the replay drive the motors
            if (!driveTrain.replayStep(Constants.REPLAY_SPEED)) {
                // Replay just finished — fall through to normal drive next loop
            }
        } else {
            // Normal teleop driving
            double forward = -oi.getLeftDriveY();  // Forward/backward (inverted)
            double turn = oi.getLeftDriveX();       // Left/right turning

            driveTrain.driveArcade(turn, forward);

            // Capture encoder/power data while recording
            driveTrain.samplePath();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop motors when command ends
        driveTrain.driveArcade(0, 0);
    }
}
