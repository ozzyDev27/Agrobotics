package frc.robot.gamepad;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class OI {
    
    private Joystick drivePad;

    public OI() {
        drivePad = new Joystick(GamepadConstants.USB_PORT);
    }

    private double applyDeadzone(double value, double deadzone) {
        return (Math.abs(value) < deadzone) ? 0.0 : value;
    }

    public double getRightDriveY() {
        return applyDeadzone(drivePad.getRawAxis(GamepadConstants.RIGHT_ANALOG_Y), Constants.DEADZONE);
    }

    public double getRightDriveX() {
        return applyDeadzone(drivePad.getRawAxis(GamepadConstants.RIGHT_ANALOG_X), Constants.DEADZONE);
    }

    public double getLeftDriveY() {
        return applyDeadzone(drivePad.getRawAxis(GamepadConstants.LEFT_ANALOG_Y), Constants.DEADZONE);
    }

    public double getLeftDriveX() {
        return applyDeadzone(drivePad.getRawAxis(GamepadConstants.LEFT_ANALOG_X), Constants.DEADZONE);
    }

    public boolean getDriveRightTrigger() {
        return drivePad.getRawButton(GamepadConstants.RIGHT_TRIGGER);
    }

    public boolean getDriveRightBumper() {
        return drivePad.getRawButton(GamepadConstants.RIGHT_BUMPER);
    }

    public boolean getDriveLeftTrigger() {
        return drivePad.getRawButton(GamepadConstants.LEFT_TRIGGER);
    }

    public boolean getDriveLeftBumper() {
        return drivePad.getRawButton(GamepadConstants.LEFT_BUMPER);
    }

    public boolean getDriveXButton() {
        return drivePad.getRawButton(GamepadConstants.SHARE_BUTTON);
    }

    public boolean getDriveYButton() {
        return drivePad.getRawButton(GamepadConstants.TRIANGLE_BUTTON);
    }

    public boolean getDriveBButton() {
        return drivePad.getRawButtonPressed(GamepadConstants.CIRCLE_BUTTON);
    }

    public boolean getDriveAButton() {
        return drivePad.getRawButtonPressed(GamepadConstants.X_BUTTON);
    }

    public boolean getDriveBackButton() {
        return drivePad.getRawButton(GamepadConstants.SHARE_BUTTON);
    }

    public boolean getDriveStartButton() {
        return drivePad.getRawButton(GamepadConstants.OPTIONS_BUTTON);
    }

    public boolean getDriveRightAnalogButton() {
        return drivePad.getRawButton(GamepadConstants.RIGHT_ANALOG_BUTTON);
    }

    public boolean getDriveLeftAnalogButton() {
        return drivePad.getRawButton(GamepadConstants.LEFT_ANALOG_BUTTON);
    }

    public int getPOV() {
        return drivePad.getPOV();
    }
}