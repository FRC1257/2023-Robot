package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SnailController extends XboxController {

    public SnailController(int port) {
        super(port);
    }

    public JoystickButton getButton(int id) {
        return new JoystickButton(this, id);
    }

    public Trigger getTrigger(boolean leftHand) {
        return new Trigger(getTriggerSupplier(leftHand));
    }

    public BooleanSupplier getTriggerSupplier(boolean left) {
        return () -> {
            if (left) return getLeftTriggerAxis() > 0.5; 
            else return getRightTriggerAxis() > 0.5;
        };
    }

    public double getElevatorSpeed() {
        return getLeftTriggerAxis() - getRightTriggerAxis();
    }

    public void startRumble() {
        setRumble(RumbleType.kLeftRumble, 1);
        setRumble(RumbleType.kRightRumble, 1);
    }

    public void stopRumble() {
        setRumble(RumbleType.kLeftRumble, 0);
        setRumble(RumbleType.kRightRumble, 0);
    }

    public enum DPad {
        UP,
        RIGHT,
        DOWN,
        LEFT
    }

    public Trigger getDPad(DPad dpad) {
        int angle;
        switch(dpad) {
            case UP:
                angle = 0;
                break;
            case RIGHT:
                angle = 90;
                break;
            case DOWN:
                angle = 180;
                break;
            case LEFT:
                angle = 270;
                break;
            default:
                angle = 0;
        }

        return new Trigger(() -> (this.getPOV() == angle));
    }

    public double getDriveForward() {
        if (getAButton()) {
            return -applyDeadband(getLeftY());
        } else if (getRightBumper()) {
            return -applyDeadband(getRightY());
        } else if (getTriggerSupplier(true).getAsBoolean()) {
            return -applyDeadband(getLeftY());
        }
        return 0;
    }

    public double getDriveTurn() {
        if (getAButton()) {
            return applyDeadband(getLeftX());
        } else if (getRightBumper()) {
            return applyDeadband(getLeftX());
        } else if (getTriggerSupplier(true).getAsBoolean()) {
            return applyDeadband(getRightX());
        }
        return 0;
    }
    //these two commands work together to get input from the joystick to control the robot
    //since getLeftBumper is true for driveforward and driveturn, it'll make it so tha tthe
    //left y joystick controls the full movement

    public static double applyDeadband(double value) {
        if (Math.abs(value) < 0.08) return 0;
        else return value;
    }
}