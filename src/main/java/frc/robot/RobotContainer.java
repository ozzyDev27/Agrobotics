package frc.robot;

import frc.robot.commands.Teleop;
import frc.robot.gamepad.OI;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  
  public static DriveTrain driveTrain;
  public static OI oi;

  public RobotContainer() {
    driveTrain = new DriveTrain();
    oi = new OI();
    
    // Set default command for drivetrain
    driveTrain.setDefaultCommand(new Teleop());
  }
}
