package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CoolController extends GenericHID {

    public CoolController(int port) {
        super(port);
        //TODO Auto-generated constructor stub
    }

    public enum Joysticks {
        PITCH(1),
        ROLL(0),
        YAW(4),
        X_JOY(2),
        Y_JOY(3);
    
        private final int value;
    
        Joysticks(int value) {
            this.value = value;
        }
    
        public int getValue() {
            return value;
        }
    }

    public enum Buttons {
        TRIGGER(0),
        A(1),
        TOP_LEFT(3),
        TOP_RIGHT(4),
        BLACK_TRIGGER(5),
        BOTTOM_RIGHT(6),
        LEFT_JOY(7),
        RIGHT_JOY(8),
        LEFT_TOP(9),
        RIGHT_TOP(10),
        RIGHT_DPAD(11),
        UP_DPAD(10),
        DOWN_DPAD(12),
        LEFT_DPAD(13);
    
        private final int value;
    
        Buttons(int value) {
            this.value = value;
        }
    
        public int getValue() {
            return value;
        }
    }
    
    public double getDriveForward() {
        return applyDeadband(getRawAxis(Joysticks.PITCH.value));
    }

    public double getDriveTurn() {
        return applyDeadband(getRawAxis(Joysticks.ROLL.value)) + applyDeadband(getRawAxis(Joysticks.YAW.value));
    }

    public Trigger getAButton() {
        return new Trigger(() -> getRawButton(Buttons.A.value));
    }

    public Trigger getBlackTrigger() {
        return new Trigger(() -> getRawButton(Buttons.BLACK_TRIGGER.value));
    }

    public Trigger getTrigger() {
        return new Trigger(() -> getRawButton(Buttons.TRIGGER.value));
    }

    public Trigger getButton(int button) {
        return new Trigger(() -> getRawButton(button));
    }

    public static double applyDeadband(double value) {
        if (Math.abs(value) < 0.08) return 0;
        else return value;
    }
    
}
