package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import static frc.robot.Constants.ElectricalLayout.*;

public class PneumaticExample extends SnailSubsystem {
    public DoubleSolenoid solenoid;

    public enum States {
        EXTEND, RETRACT, OFF
    }

    private States state = States.EXTEND;

    public PneumaticExample() {
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DOUBLE_SOLENOID_FORWARD, DOUBLE_SOLENOID_REVERSE);
    }

    @Override
    public void update() {
        switch (state) {
            case EXTEND:
                solenoid.set(DoubleSolenoid.Value.kForward);
                break;
            case RETRACT:
                solenoid.set(DoubleSolenoid.Value.kReverse);
                break;
            case OFF:
                solenoid.set(DoubleSolenoid.Value.kOff);
                break;
        }
        
    }

    public void extend() {
        state = States.EXTEND;
    }

    public void retract() {
        state = States.RETRACT;
    }

    public void off() {
        state = States.OFF;
    }

    // nothing to display for these really
    @Override
    public void displayShuffleboard() {}

    @Override
    public void tuningInit() {}

    @Override
    public void tuningPeriodic() {}
    
}
