package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.frc.robot.Constants.*;

public class Claw extends SnailSubsystem {
    private CANSparkMax motorLeft;
    private CANSparkMax motorRight;
    private DoubleSolenoid solenoid;
    
    public enum RollerState {
      INTAKING,
      EJECTING,
      NEUTRAL
    }
    
    public enum ClawState {
      CUBEINTAKE,
      CONEINTAKE
    }
    
    //cube and cone states?
    private RollerState;
    private ClawState;
    
    public ClawAlt() {
        motorLeft = new CANSparkMax(CLAW_MOTOR_LEFT, MotorType.kBrushless);
        motorRight = new CANSparkMax(CLAW_MOTOR_RIGHT, MotorType.kBrushless);
        motorInit(motorLeft);
        motorInit(motorRight);
        motorRight.follow(motorLeft, true);
        rollerstate = RollerState.NEUTRAL;
        
        solenoid = new DoubleSolenoid(CLAW_FORWARD_ID, CLAW_REVERSE_ID);
        clawstate = ClawState.CUBEINTAKE;
    }
    private void motorInit(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
    }
    @Override
    public void update() {
        switch(rollerstate) {
            case NEUTRAL:
                motorLeft.set(0.0);
                break;
            case INTAKING:
                motorLeft.set(0.85);
                break;
            case EJECTING:
                motorLeft.set(-0.85);
                break;
        }
        
        switch(clawstate) {
            case CUBEINTAKE:
                solenoid.set(Value.kReverse);
                break;
            case CONEINTAKE:
                solenoid.set(Value.kForward);
                break;
        }
    }

    @Override
    public void displayShuffleboard() {
        SmartDashboard.putNumber("Left Motor Current", motorLeft.getOutputCurrent());
        SmartDashboard.putNumber("Right Motor Current", motorRight.getOutputCurrent());
    }

    @Override
    public void tuningInit() {

    }

    @Override
    public void tuningPeriodic() {

    }

    public void neutral() {
        state = State.NEUTRAL;
    }

    public void eject() {
        state = State.EJECTING;
    }

    public void intake() {
        state = State.INTAKING;
    }
    
    public void cubeintake() {
        state = State.CUBEINTAKE;
    }
    
    public void coneintake() {
        state = State.CONEINTAKE;
    }
    
    public RollerState getRollerState() {
        return rollerstate;
    }
    public ClawState getClawState() {
        return clawstate;
    }
}
