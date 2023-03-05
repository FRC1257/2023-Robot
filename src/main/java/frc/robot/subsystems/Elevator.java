package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.ElectricalLayout.ELEVATOR_MOTOR_ID;
import static frc.robot.Constants.ElevatorSpeed.ELEVATOR_EXTEND_SPEED;
import static frc.robot.Constants.ElevatorSpeed.ELEVATOR_IDLE_SPEED;
import static frc.robot.Constants.ElevatorSpeed.ELEVATOR_RETRACT_SPEED;

public class Elevator extends SnailSubsystem{

    private CANSparkMax elevatorMotor;

    public Elevator() {
        elevatorMotor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    }

    public enum State {
        EXTENDED,
        RETRACTED,
        IDLE;
    }

    private State elevatorState = State.RETRACTED; 

    @Override
    public void update() {
        switch(elevatorState) {
            case RETRACTED:
                elevatorMotor.set(ELEVATOR_RETRACT_SPEED);
                break;
            case EXTENDED:
                elevatorMotor.set(ELEVATOR_EXTEND_SPEED);
                break;
            case IDLE:
                elevatorMotor.set(ELEVATOR_IDLE_SPEED);
                break;
        }
    }

    public void extended() {
        elevatorState = State.EXTENDED;
    }

    public void retract() {
        elevatorState = State.RETRACTED;
    }

    public void idle() {
        elevatorState = State.IDLE;
    }

    public State getState() {
        return elevatorState;
    }

    @Override
    public void displayShuffleboard() {
        SmartDashboard.putNumber("Elevator Motor Speed", elevatorMotor.get());
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
