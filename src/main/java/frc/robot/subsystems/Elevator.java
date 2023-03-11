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
import frc.robot.util.TunableNumber;

public class Elevator extends SnailSubsystem{

    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    private DigitalInput limitSwitch;
    private double speed;
    private double setpoint;
    private boolean isPIDFinished;

    private TunableNumber p = new TunableNumber("Pivot Arm P", ELEVATOR_PID[0]);
    private TunableNumber i = new TunableNumber("Pivot Arm I", ELEVATOR_PID[1]);
    private TunableNumber d = new TunableNumber("Pivot Arm D", ELEVATOR_PID[2]);
    private TunableNumber ff = new TunableNumber("Pivot Arm FF", ELEVATOR_PID[3]);
    private TunableNumber maxOutput = new TunableNumber("Pivot Arm IZ", ELEVATOR_PID_MAX_OUTPUT);


    public enum State {
        MANUAL,
        PID;
    }

    private State elevatorState = State.MANUAL; 

    public Elevator() {
        elevatorMotor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        pidController = elevatorMotor.getPIDController();

        pidController.setP(p.get());
        pidController.setI(i.get());
        pidController.setD(d.get());
        pidController.setFF(ff.get());
        pidController.setOutputRange(-maxOutput.get(), maxOutput.get());

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
        p.updateFunction(() -> pidController.setP(p.get()));
        i.updateFunction(() -> pidController.setI(i.get()));
        d.updateFunction(() -> pidController.setD(d.get()));
        ff.updateFunction(() -> pidController.setFF(ff.get()));
        maxOutput.updateFunction(() -> pidController.setOutputRange(-maxOutput.get(), maxOutput.get()));
    }
    
}
