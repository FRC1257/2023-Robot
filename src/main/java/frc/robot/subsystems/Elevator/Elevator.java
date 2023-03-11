package frc.robot.subsystems.Elevator;
import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.SnailSubsystem;

public class Elevator extends SnailSubsystem{

    private ElevatorIO io;
    private MechanismLigament2d ElevatorMechanism;

    public Elevator(ElevatorIO io){
       this.io = io;
    }

    @Override
    public void update() {
        io.updateIO();
    }

    public void setPosition(double setpoint) {
        io.setPosition(setpoint);
    }

    @Override
    public void displayShuffleboard() {
        io.displayShuffleboardIO();
    }

    @Override
    public void tuningInit() {
        io.tuningInitIO();
    }

    @Override
    public void tuningPeriodic() {
        io.tuningPeriodicIO();
    }

    public void manualControl(double newSpeed) {
        io.manual(newSpeed);
    }

    public ElevatorIO.State getState() { return io.getState(); }

    public void setMechanism(MechanismLigament2d mechanism) {
        ElevatorMechanism = mechanism;
    }

    public MechanismLigament2d append(MechanismLigament2d mechanism) {
        return ElevatorMechanism.append(mechanism);
    }

    public MechanismLigament2d getArmMechanism() {
        return new MechanismLigament2d("Elevator", ELEVATOR_ARM_LENGTH, 0, 5, new Color8Bit(Color.kAqua));
    }
    
}
