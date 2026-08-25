package example.robot.controlboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import example.robot.Constants.OIConstants;

/**
 * Driver/operator XboxController'lari sarmalar; RobotContainer ve factories buradaki
 * isimlendirilmis erisimcileri kullanir, ham controller portlarina dogrudan dokunmaz.
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

    /** Y -> gyro/yonelim sifirla */
    public Trigger zeroHeading() {
        return new JoystickButton(driver, XboxController.Button.kY.value);
    }

    /** B -> intake toggle */
    public Trigger intakeToggle() {
        return new JoystickButton(driver, XboxController.Button.kB.value);
    }

    /** LT basili tutulunca alma rulosu doner */
    public Trigger intakeRoller() {
        return new Trigger(() -> driver.getLeftTriggerAxis() > 0.5);
    }

    /** RT -> hizala + at */
    public Trigger alignAndShoot() {
        return new Trigger(() -> driver.getRightTriggerAxis() > 0.5);
    }

    /** RB -> manuel atis */
    public Trigger manualShoot() {
        return new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    }

    public XboxController operator() {
        return operator;
    }
}
