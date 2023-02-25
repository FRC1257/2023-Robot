package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static frc.robot.Constants.ElectricalLayout.*;

public class Elevator extends SnailSubsystem{

    private DoubleSolenoid elevatorSolenoidRight;
    private DoubleSolenoid elevatorSolenoidLeft;

    public Elevator() {
        elevatorSolenoidLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, EXTENDER_LEFT_FORWARD_ID, EXTENDER_LEFT_REVERSE_ID);
        elevatorSolenoidRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, EXTENDER_RIGHT_FORWARD_ID, EXTENDER_RIGHT_REVERSE_ID);
    }

    public enum State {
        EXTENDED,
        RETRACTED;
    }

    private State elevatorState = State.RETRACTED; 

    @Override
    public void update() {
        switch(elevatorState) {
            case RETRACTED:
                elevatorSolenoidLeft.set(Value.kReverse);
                elevatorSolenoidRight.set(Value.kReverse);
                break;
            case EXTENDED:
                elevatorSolenoidLeft.set(Value.kForward);
                elevatorSolenoidRight.set(Value.kForward);
                break;
        }
    }

    public void extended() {
        elevatorState = State.EXTENDED;
    }

    public void retract() {
        elevatorState = State.RETRACTED;
    }

    public State getState() {
        return elevatorState;
    }

    @Override
    public void displayShuffleboard() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void tuningInit() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void tuningPeriodic() {
        // TODO Auto-generated method stub
        
    }
    
}
