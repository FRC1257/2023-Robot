package frc.robot.subsystems;
import static frc.robot.Constants.ElectricalLayout.ELEVATOR_MOTOR_ID;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.TunableNumber;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.NEO_CURRENT_LIMIT;

import com.revrobotics.SparkMaxRelativeEncoder;


public class Elevator extends SnailSubsystem{

    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    private DigitalInput limitSwitch; 
    private double speed;
    private double setpoint;
    private boolean isPIDFinished;

    private TunableNumber p = new TunableNumber("Elevator P", ELEVATOR_PID[0]);
    private TunableNumber i = new TunableNumber("Elevator I", ELEVATOR_PID[1]);
    private TunableNumber d = new TunableNumber("Elevator D", ELEVATOR_PID[2]);
    private TunableNumber ff = new TunableNumber("Elevator FF", ELEVATOR_PID[3]);

    public enum State {
        MANUAL,
        PID
    }

    private State elevatorState = State.MANUAL; 
    
    public Elevator() {
        SmartDashboard.putBoolean("Motor mode", false);
        
        elevatorMotor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        // pivotWristMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setIdleMode(IdleMode.kCoast);
        elevatorMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        /* elevatorMotor.setInverted(true);

        //in the forward direction, it will stop at 0.5 inches
        elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, ELEVATOR_SETPOINT_RETRACT);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, ELEVATOR_SETPOINT_EXTEND);
        //in reverse direction, it will stop at 36 inches
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true); //always true */
        pidController = elevatorMotor.getPIDController();

        pidController.setP(p.get());
        pidController.setI(i.get());
        pidController.setD(d.get());
        pidController.setFF(ff.get());
        pidController.setOutputRange(-1, 1);

        encoder = elevatorMotor.getEncoder();
        encoder.setPositionConversionFactor(ELEVATOR_REV_TO_POS_FACTOR);
        encoder.setVelocityConversionFactor(ELEVATOR_REV_TO_POS_FACTOR / 60);
        // encoder.setInverted(true);
        encoder.setPosition(0.6);

        limitSwitch = new DigitalInput(0); 
    }


    @Override
    public void update() {
        // negative speed moves it up
        if (encoder.getPosition() <= -ELEVATOR_SETPOINT_EXTEND && speed < 0.0) {
            elevatorMotor.set(0);
            return;
        } else if (encoder.getPosition() >= -ELEVATOR_SETPOINT_RETRACT && speed > 0.0) {
            elevatorMotor.set(0);
            return;
        } 

        SmartDashboard.putBoolean("Elevator Extend", encoder.getPosition() <= -ELEVATOR_SETPOINT_EXTEND/*  && speed < 0.0*/);
        SmartDashboard.putBoolean("Elevator Bottom", encoder.getPosition() >= -ELEVATOR_SETPOINT_RETRACT /*&& speed > 0.0*/);

        switch(elevatorState) {
            case MANUAL:
                elevatorMotor.set(speed);
                break;
            case PID:
                pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
                /* if (Math.abs(encoder.getPosition() - setpoint) < ELEVATOR_PID_TOLERANCE ) {
                    endPID();
                } */
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
        SmartDashboard.putNumber("Elevator Encoder", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);
        SmartDashboard.putString("Elevator State", elevatorState.toString());
        SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
        SmartDashboard.putString("Elevator Brake", elevatorMotor.getIdleMode().toString());

    }

    @Override
    public void tuningInit() {
        p.reset();
        i.reset();
        d.reset();
        ff.reset();
    }

    @Override
    public void tuningPeriodic() {
        p.updateFunction(() -> pidController.setP(p.get()));
        i.updateFunction(() -> pidController.setI(i.get()));
        d.updateFunction(() -> pidController.setD(d.get()));
        ff.updateFunction(() -> pidController.setFF(ff.get()));

        if (SmartDashboard.getBoolean("Motor mode", false) && elevatorMotor.getIdleMode() != IdleMode.kBrake) {
            elevatorMotor.setIdleMode(IdleMode.kBrake);
        } else if (!SmartDashboard.getBoolean("Motor mode", false) && elevatorMotor.getIdleMode() != IdleMode.kCoast) {
            elevatorMotor.setIdleMode(IdleMode.kCoast);
        }
    }
    
    public boolean atSetpoint() {
        return Math.abs(encoder.getPosition() - setpoint) < ELEVATOR_PID_TOLERANCE;
    }
}
