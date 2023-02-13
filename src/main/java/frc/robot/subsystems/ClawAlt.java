package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.frc.robot.Constants.*;

public class ClawAlt extends SnailSubsystem {
    private CANSparkMax motorLeft;
    private CANSparkMax motorRight;
    public enum State {
      INTAKING,
      EJECTING,
      NEUTRAL,
    }
    //cube and cone states?
    private State state = State.NEUTRAL;
    
    public ClawAlt() {
        motorLeft = new CANSparkMax(CLAW_MOTOR_LEFT, MotorType.kBrushless);
        motorRight = new CANSparkMax(CLAW_MOTOR_RIGHT, MotorType.kBrushless);
        motorInit(motorLeft);
        motorInit(motorRight);
        motorRight.follow(motorLeft, true);
    }
    private void motorInit(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
    }
    @Override
    public void update() {
        switch(state) {
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
    }

    @Override
    public void displayShuffleboard() {
        
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
    
    public State getState() {
        return state;
    }
}
