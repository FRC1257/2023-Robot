package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.NEO_550_CURRENT_LIMIT;

public class PivotArm extends SnailSubsystem {
    private CANSparkMax armMotor;

    public enum State {
        MANUAL,
        PID
    }

    private State state = State.MANUAL;
    private double speed = 0;

    public PivotArm() {
        armMotor = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
        armMotor.restoreFactoryDefaults();
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
    }

    @Override
    public void update() {
        switch (state) {
            case MANUAL: {
                armMotor.set(speed);
            }
            case PID: {
                // TODO: Add PID
            }
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

    public void manualControl(double newSpeed) {
        speed = newSpeed;
        state = State.MANUAL;
    }

    public State getState() { return state; }
}
