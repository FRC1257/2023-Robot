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
    private CANEncoder encoder; 
   public enum State {
        MANUAL,
        PID,
    }

    private State state = State.MANUAL;
    private double speed;

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
      encoder = new CANEncoder(motorLeft);
}
  
   @Override
    public void update() {
        switch(state) {
            case MANUAL:
                motorLeft.set(speed);
                break;
        }
    }
  
   public void displayShuffleboard() {

    }

    public void tuningInit() {

    }

    public void tuningPeriodic() {

    }
  
   public void manualControl(double speed){
        this.speed = speed;
        state = State.MANUAL;
    }

    public State getState() {
        return state;
    }
}
