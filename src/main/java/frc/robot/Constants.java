package frc.robot;
import edu.wpi.first.wpilibj.SPI;

public final class Constants {
    
    public static final int TITAN_ID = 42;
    public static final int LEFT_FRONT = 0;
    public static final int LEFT_BACK = 1;
    public static final int RIGHT_FRONT = 2;
    public static final int RIGHT_BACK = 3;

    public static final double DEADZONE = 0.06;

    public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;
    public static final double REPLAY_SPEED = 1;

    private static final double wheelRadius = 62.5;
    private static final double pulsePerRevolution = 1464;
    private static final double gearRatio = 1.0;
    private static final double encoderPulseRatio = pulsePerRevolution * gearRatio;
    public static final double DIST_PER_TICK = (Math.PI * 2 * wheelRadius) / encoderPulseRatio;
}
