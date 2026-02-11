package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
        // Get joystick values
        double forward = -oi.getLeftDriveY();  // Forward/backward (inverted)
        double turn = oi.getLeftDriveX();      // Left/right turning

        // Drive the robot
        driveTrain.driveArcade(turn, forward);
    }

    @Override
    public boolean isFinished() {
        return false;  // Command never finishes on its own
    }

    @Override
    public void end(boolean interrupted) {
        // Stop motors when command ends
        driveTrain.driveArcade(0, 0);
    }
}
