package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    
    private TitanQuad leftBack;
    private TitanQuad leftFront;
    private TitanQuad rightBack;
    private TitanQuad rightFront;

    public AHRS navX;

    public DriveTrain() { 
        leftBack = new TitanQuad(Constants.TITAN_ID, Constants.LEFT_BACK);
        leftFront = new TitanQuad(Constants.TITAN_ID, Constants.LEFT_FRONT);
        rightBack = new TitanQuad(Constants.TITAN_ID, Constants.RIGHT_BACK);
        rightFront = new TitanQuad(Constants.TITAN_ID, Constants.RIGHT_FRONT);

        Timer.delay(1);

        navX = new AHRS(SPI.Port.kMXP);

        // Invert right side motors
        rightBack.setInverted(true);
        rightFront.setInverted(true);
    }

    // Simple arcade drive - x is turn, y is forward/backward
    public void driveArcade(double x, double y) {
        // Clamp values between -1 and 1
        x = Math.max(-1.0, Math.min(1.0, x));
        y = Math.max(-1.0, Math.min(1.0, y));
        
        // Left side: forward + turn
        leftBack.set(y + x);
        leftFront.set(y + x);
        
        // Right side: forward - turn
        rightBack.set(y - x);
        rightFront.set(y - x);
    }

    // Reset the gyro heading to zero
    public void resetGyro() {
        navX.zeroYaw();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
