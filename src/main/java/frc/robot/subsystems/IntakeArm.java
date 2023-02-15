package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.IntakeArm.*;
import static frc.robot.Constants.NEO_550_CURRENT_LIMIT;

public class IntakeArm extends SnailSubsystem {

    private CANSparkMax motorLeft;
    private CANSparkMax motorRight;
    private SparkMaxPIDController pidController;
    private SparkMaxRelativeEncoder encoder; 
    private DigitalInput bumpSwitch;
    
   public enum State {
        MANUAL,
        PID,
    }

    private State state = State.MANUAL;
    private double speed;
    private boolean isPIDFinished;
  public IntakeArm() {
      motorLeft = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
      motorLeft.restoreFactoryDefaults();
      motorLeft.setIdleMode(IdleMode.kBrake);
      motorLeft.setSmartCurrentLimit(NEO_CURRENT_LIMIT); // in amps
      
      motorRight = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
      motorRight.restoreFactoryDefaults();
      motorRight.setIdleMode(IdleMode.kBrake);
      motorRight.setSmartCurrentLimit(NEO_CURRENT_LIMIT); // in amps
      
      motorRight.follow(motorLeft);
      
      pidController = motorLeft.getPIDController();
      pidController.setP(INTAKE_ARM_PID[0]);
      pidController.setI(INTAKE_ARM_PID[1]);
      pidController.setD(INTAKE_ARM_PID[2]);
      pidController.setFF(INTAKE_ARM_PID[3]);
      pidController.setOutputRange(-INTAKE_ARM_PID_MAX_OUTPUT, INTAKE_ARM_PID_MAX_OUTPUT);
    
      
      encoder = new SparkMaxRelativeEncoder(motorLeft);
      encoder.setPositionConversionFactor(INTAKE_ARM_GEAR_FACTOR);
      encoder.setVelocityConversionFactor(INTAKE_ARM_VELOCITY_FACTOR);
      encoder.setPosition(0.0);
      
      bumpSwitch = new DigitalInput(INTAKE_BUMP_SWITCH_ID);
      isPIDFinished = false;
}
  
   @Override
    public void update() {
        switch(state) {
            case MANUAL:
                motorLeft.set(speed);
                break;
            case PID:
                pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
                if (bumpSwitch.get()) {
                    endPID();
                }
                break;
        }
    }
  
   public void displayShuffleboard() {
        SmartDashboard.putNumber("Motor Speed", encoder.getVelocity());
        SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
        SmartDashboard.putNumber("Setpoint", setpoint);
        
    }

    public void tuningInit() {

    }

    public void tuningPeriodic() {

    }
    
   public void endPID() {
   	    state = state.MANUAL;
        isPIDFinished = true;
   }
   public void manualControl(double speed){
        this.speed = speed;
        state = State.MANUAL;
    }
    
   public void setPosition(double setpoint) {
        state = State.PID;
        if (!isPIDFinished) this.setpoint = setpoint;
   }

    public State getState() {
        return state;
    }
}
