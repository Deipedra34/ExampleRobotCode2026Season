package example.robot.controlboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import example.robot.Constants.OIConstants;

/**
 * Wraps the driver/operator XboxControllers. RobotContainer and the factories go through
 * the named accessors here instead of poking at raw controller ports directly.
 */
public class ControlBoard {

    private final XboxController driver   = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    private final XboxController operator = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

    public double getDriveX() {
        return -MathUtil.applyDeadband(driver.getLeftY(), OIConstants.DEADBAND);
    }

    public double getDriveY() {
        return -MathUtil.applyDeadband(driver.getLeftX(), OIConstants.DEADBAND);
    }

    public double getDriveRotation() {
        return -MathUtil.applyDeadband(driver.getRightX(), OIConstants.DEADBAND);
    }

    /** Y -> reset gyro/heading */
    public Trigger zeroHeading() {
        return new JoystickButton(driver, XboxController.Button.kY.value);
    }

    /** B -> toggle intake */
    public Trigger intakeToggle() {
        return new JoystickButton(driver, XboxController.Button.kB.value);
    }

    /** intake roller spins while LT is held */
    public Trigger intakeRoller() {
        return new Trigger(() -> driver.getLeftTriggerAxis() > 0.5);
    }

    /** RT -> aim and shoot */
    public Trigger alignAndShoot() {
        return new Trigger(() -> driver.getRightTriggerAxis() > 0.5);
    }

    /** RB -> manual shot */
    public Trigger manualShoot() {
        return new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    }

    public XboxController operator() {
        return operator;
    }
}
