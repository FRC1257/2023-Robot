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
        leftArmMotor = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
        leftArmMotor.restoreFactoryDefaults();
        leftArmMotor.setIdleMode(IdleMode.kBrake);
        leftArmMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);

        rightArmMotor = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
        rightArmMotor.restoreFactoryDefaults();
        rightArmMotor.setIdleMode(IdleMode.kBrake);
        rightArmMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);

        rightArmMotor.follow(leftArmMotor);
    }

    @Override
    public void update() {
        switch (state) {
            case MANUAL: {
                armMotor.set(speed);
                break;
            }
            case PID: {
                pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
                if (bumpSwitch.get()) {
                    endPID();
                }
                break;
            }
        }
    }

    @Override
    public void displayShuffleboard() {
        SmartDashboard.putNumber("Motor Speed", encoder.getVelocity());
        SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
        SmartDashboard.putNumber("Setpoint", setpoint);
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

    public State getState() {
        return state;
    }
}
