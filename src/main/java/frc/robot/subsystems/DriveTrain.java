package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
 

    // Simple arcade drive - x is turn, y is forward/backward
    public void driveArcade(double x, double y) {
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
        // leftBack.set(3);
        // leftFront.set(3);
        // rightBack.set(0);
        // rightFront.set(0);
        // System.out.println("hi");
    }
}
