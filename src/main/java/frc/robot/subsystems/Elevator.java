package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.Elevator.*;

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
    private MechanismLigament2d elevatorMechanism;

    @Override
    public void update() {
        switch(elevatorState) {
            case RETRACTED:
                elevatorSolenoidLeft.set(Value.kReverse);
                elevatorSolenoidRight.set(Value.kReverse);
                elevatorMechanism.setLength(RETRACT_LENGTH);
                break;
            case EXTENDED:
                elevatorSolenoidLeft.set(Value.kForward);
                elevatorSolenoidRight.set(Value.kForward);
                elevatorMechanism.setLength(EXTEND_LENGTH);
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

    public MechanismLigament2d getElevatorMechanism() {
        return new MechanismLigament2d("elevator", RETRACT_LENGTH, ANGLE);
    }
    
    public void setElevatorMechanism(MechanismLigament2d elevatorMechanism) {
        this.elevatorMechanism = elevatorMechanism;
    }
    
    public MechanismLigament2d append(MechanismLigament2d object) {
        return elevatorMechanism.append(object);
    }
    
}
