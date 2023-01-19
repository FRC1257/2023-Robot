package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.Claw.*;
public class Claw extends SnailSubsystem {

    private final DoubleSolenoid rightSolenoid;
    private DoubleSolenoid leftSolenoid;
    public enum State {
       CLOSED,
       OPEN
    }

    State state = State.OPEN;

    public Claw() {
        leftSolenoid = new DoubleSolenoid(LEFTCLAWFORWARD_ID, LEFTCLAWREVERSE_ID);
        rightSolenoid = new DoubleSolenoid(RIGHTCLAWFORWARD_ID, RIGHTCLAWREVERSE_ID);
    }

    @Override
    public void update() {
        switch(state) {
            case CLOSED:
                leftSolenoid.set(Value.kForward);
                rightSolenoid.set(Value.kForward);
                break;
            case OPEN:
                leftSolenoid.set(Value.kReverse);
                rightSolenoid.set(Value.kReverse);
                break;
        }
    }

    public void displayShuffleboard(){

    }
    public void tuningInit(){

    }
    public void tuningPeriodic(){

    }

    public void open() {
        state = State.OPEN;
    }

    public void close() {
        state = State.CLOSED;
    }

    public State getState() {
        return state;
    }
}
