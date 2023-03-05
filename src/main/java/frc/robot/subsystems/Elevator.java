package frc.robot.subsystems;
import static frc.robot.Constants.ElectricalLayout.ELEVATOR_MOTOR_ID;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_PID;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_PID_MAX_OUTPUT;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_REV_TO_POS_FACTOR;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_PID_TOLERANCE;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SnailSubsystem{

    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    private DigitalInput limitSwitch;
    private double speed;
    private double setpoint;
    private boolean isPIDFinished;

    public enum State {
        MANUAL,
        PID;
    }

    private State elevatorState = State.MANUAL; 

    public Elevator() {
        elevatorMotor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        pidController = elevatorMotor.getPIDController();

        pidController.setP(ELEVATOR_PID[0]);
        pidController.setI(ELEVATOR_PID[1]);
        pidController.setD(ELEVATOR_PID[2]);
        pidController.setFF(ELEVATOR_PID[3]);
        pidController.setOutputRange(-ELEVATOR_PID_MAX_OUTPUT, ELEVATOR_PID_MAX_OUTPUT);

        encoder = elevatorMotor.getEncoder();
        encoder.setPositionConversionFactor(ELEVATOR_REV_TO_POS_FACTOR);
        encoder.setVelocityConversionFactor(ELEVATOR_REV_TO_POS_FACTOR / 60);
        encoder.setPosition(0.0);

        limitSwitch = new DigitalInput(ELEVATOR_MOTOR_ID);
    }


    @Override
    public void update() {
        switch(elevatorState) {
            case MANUAL:
                elevatorMotor.set(speed);
                break;
            case PID:
                pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
                if (Math.abs(encoder.getPosition() - setpoint) < ELEVATOR_PID_TOLERANCE ) {
                    endPID();
                }
                break;
        }
    }

    public void endPID() {
        elevatorState = State.MANUAL;
    }

    public void manual(double speed){
        this.speed = speed;
        elevatorState = State.MANUAL;
    }

    public void setPosition(double setpoint) {
        elevatorState = State.PID;
        if (!isPIDFinished) this.setpoint = setpoint;
   }

    public State getState() {
        return elevatorState;
    }

    public double getPosition() {
        return encoder.getPosition();
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
