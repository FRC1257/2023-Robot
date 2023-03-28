package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.IntakeArmConstants.*;
import static frc.robot.Constants.NEO_CURRENT_LIMIT;
import static frc.robot.Constants.ElectricalLayout.INTAKE_ARM_MOTOR_LEFT_ID;
import static frc.robot.Constants.ElectricalLayout.INTAKE_BUMP_SWITCH_ID;

public class IntakeArm extends SnailSubsystem {

    private CANSparkMax motorLeft;
    // private CANSparkMax motorRight;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder; 
    private DigitalInput bumpSwitch;
    private double setpoint;
    public enum State {
        MANUAL,
        PID,
    }

    private State state;
    private double speed;
    private boolean isPIDFinished;

  public IntakeArm() {
      motorLeft = new CANSparkMax(INTAKE_ARM_MOTOR_LEFT_ID, MotorType.kBrushless);
      // motorRight = new CANSparkMax(INTAKE_ARM_MOTOR_RIGHT_ID, MotorType.kBrushless);
      motorInit(motorLeft);
      /* motorInit(motorRight);
      motorRight.follow(motorLeft); */
      
      pidController = motorLeft.getPIDController();
      pidController.setP(INTAKE_ARM_PID[0]);
      pidController.setI(INTAKE_ARM_PID[1]);
      pidController.setD(INTAKE_ARM_PID[2]);
      pidController.setFF(INTAKE_ARM_PID[3]);
      pidController.setOutputRange(-INTAKE_ARM_PID_MAX_OUTPUT, INTAKE_ARM_PID_MAX_OUTPUT);
    
      
      encoder = motorLeft.getEncoder();
      encoder.setPositionConversionFactor(INTAKE_ARM_GEAR_FACTOR);
      encoder.setVelocityConversionFactor(INTAKE_ARM_GEAR_FACTOR / 60);
      encoder.setPosition(0.0);
      
      state = State.MANUAL;
      bumpSwitch = new DigitalInput(INTAKE_BUMP_SWITCH_ID);
      isPIDFinished = false;
}
    private void motorInit(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
    }

   @Override
    public void update() {
        switch(state) {
            case MANUAL:
                motorLeft.set(speed);
                break;
            case PID:
                pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
                if (Math.abs(encoder.getPosition() - setpoint) < INTAKE_ARM_PID_TOLERANCE) {
                    endPID();
                }
                break;
        }
    }
  
   public void displayShuffleboard() {
        SmartDashboard.putNumber("Motor Speed", encoder.getVelocity());
        SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
        SmartDashboard.putNumber("Setpoint", setpoint);
        SmartDashboard.putBoolean("Limit Switch State", bumpSwitch.get());
        
    }

    public void tuningInit() {

    }

    public void tuningPeriodic() {

    }
    
   public void endPID() {
   	    state = State.MANUAL;
        isPIDFinished = true;
   }
   public void manual(double speed){
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

    public double getPosition() {
        return encoder.getPosition();
    }
}