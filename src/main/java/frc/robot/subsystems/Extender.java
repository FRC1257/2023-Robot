package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Extender extends SnailSubsystem{

    private DoubleSolenoid extenderSolenoidRight;
    private DoubleSolenoid extenderSolenoidLeft;

    public Extender() {
        extenderSolenoidLeft = new DoubleSolenoid(null, 0, 0);
        extenderSolenoidRight = new DoubleSolenoid(null, 0, 0)
    }

    public enum State {
        EXTENDED,
        RETRACTED;
    }

    private State extenderState = State.RETRACTED; 

    @Override
    public void update() {
    
    }

    public void extended() {
        extenderState = State.EXTENDED;
    }

    public void retract() {
        extenderState = State.RETRACTED;
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
