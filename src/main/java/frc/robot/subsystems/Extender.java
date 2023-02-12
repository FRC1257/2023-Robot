package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.ElectricalLayout;

public class Extender extends SnailSubsystem{

    private DoubleSolenoid extenderSolenoidRight;
    private DoubleSolenoid extenderSolenoidLeft;

    public Extender() {
        extenderSolenoidLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ElectricalLayout.EXTENDER_LEFT_FORWARD_ID, ElectricalLayout.EXTENDER_LEFT_REVERSE_ID);
        extenderSolenoidRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ElectricalLayout.EXTENDER_RIGHT_FORWARD_ID, ElectricalLayout.EXTENDER_RIGHT_REVERSE_ID);
    }

    public enum State {
        EXTENDED,
        RETRACTED;
    }

    private State extenderState = State.RETRACTED; 

    @Override
    public void update() {
        switch(extenderState) {
            case RETRACTED:
                extenderSolenoidLeft.set(Value.kReverse);
                extenderSolenoidRight.set(Value.kReverse);
                break;
            case EXTENDED:
                extenderSolenoidLeft.set(Value.kForward);
                extenderSolenoidRight.set(Value.kForward);
                break;
        }
    }

    public void extended() {
        extenderState = State.EXTENDED;
    }

    public void retract() {
        extenderState = State.RETRACTED;
    }

    public State getState() {
        return extenderState;
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
