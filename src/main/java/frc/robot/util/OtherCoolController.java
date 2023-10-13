package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OtherCoolController extends GenericHID {

    public OtherCoolController(int port) {
        super(port);
        //TODO Auto-generated constructor stub
    }

    public enum Joysticks {
        LEFT(0),
        RIGHT(1);
    
        private final int value;
    
        Joysticks(int value) {
            this.value = value;
        }
    
        public int getValue() {
            return value;
        }
    }

    public enum Buttons {
        UP_ONE(5),
        DOWN_ONE(6),
        UP_TWO(7),
        DOWN_TWO(8),
        UP_THREE(9),
        DOWN_THREE(10);
    
        private final int value;
    
        Buttons(int value) {
            this.value = value;
        }
    
        public int getValue() {
            return value;
        }
    }
    
    public double getElevatorPosition() {
        return -getRawAxis(Joysticks.LEFT.value);
    }

    public double getPivotPosition() {
        return getRawAxis(Joysticks.RIGHT.value);
    }

    public Trigger getUpOne() {
        return new Trigger(() -> getRawButton(Buttons.UP_ONE.value));
    }

    public Trigger getDown() {
        return new Trigger(() -> getRawButton(Buttons.DOWN_ONE.value));
    }

    public Trigger getUpTwo() {
        return new Trigger(() -> getRawButton(Buttons.UP_TWO.value));
    }
    
    public Trigger getButton(int button) {
        return new Trigger(() -> getRawButton(button));
    }
    
    public static double mapRange(double value, double fromMin, double fromMax, double toMin, double toMax) {
        // Ensure the value is within the original range
        double clampedValue = Math.min(Math.max(value, fromMin), fromMax);

        // Calculate the normalized value within the original range
        double normalizedValue = (clampedValue - fromMin) / (fromMax - fromMin);

        // Map the normalized value to the new range
        double mappedValue = normalizedValue * (toMax - toMin) + toMin;

        return mappedValue;
    }
}
