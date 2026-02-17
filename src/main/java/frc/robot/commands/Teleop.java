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
        driveTrain.resetGyro();
    }

    @Override
    public void execute() {

        if (oi.getDriveXButton()) {
            if (driveTrain.isReplaying()) {
                driveTrain.stopReplay();
            }
            driveTrain.toggleRecording();
        }

        if (oi.getDriveYButton()) {
            if (driveTrain.isReplaying()) {
                driveTrain.stopReplay();
            } else if (!driveTrain.isRecording()) {
                driveTrain.startReplay();
            }
        }

        if (driveTrain.isReplaying()) {
            driveTrain.replayStep(Constants.REPLAY_SPEED);
        } else {
            double forward = -oi.getLeftDriveY();
            double turn = oi.getLeftDriveX();

            driveTrain.driveArcade(turn, forward);
            driveTrain.samplePath();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.driveArcade(0, 0);
    }
}
