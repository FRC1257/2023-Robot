package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.PivotArm.*;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.NEO_550_CURRENT_LIMIT;

public class PivotArm extends SnailSubsystem {
    private CANSparkMax leftArmMotor, rightArmMotor;
    private RelativeEncoder leftArmEncoder;
    private State state = State.MANUAL;
    private double speed;
    private SparkMaxPIDController armPIDController;
    private double setPoint;


    public enum State {
        MANUAL,
        PID
    }

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

        leftArmEncoder = leftArmMotor.getEncoder();
        leftArmEncoder.setPositionConversionFactor(48.0 * Math.PI * 6);
        leftArmEncoder.setVelocityConversionFactor(48.0 * Math.PI * 6 / 60);

        armPIDController = leftArmMotor.getPIDController();
        armPIDController.setP(PIVOT_ARM_PID[0]);
        armPIDController.setI(PIVOT_ARM_PID[1]);
        armPIDController.setD(PIVOT_ARM_PID[2]);
        armPIDController.setOutputRange(-PIVOT_ARM_PID_MAX_OUTPUT, PIVOT_ARM_PID_MAX_OUTPUT);
    }

    @Override
    public void update() {
        switch (state) {
            case MANUAL: {
                leftArmMotor.set(speed);
                break;
            }
            case PID: {
                // send the desired setpoint to the PID controller and specify we want to use position control
                armPIDController.setReference(setPoint, ControlType.kPosition);

                // check our error and update the state if we finish
                if(Math.abs(leftArmEncoder.getPosition() - setPoint) < PIVOT_ARM_PID_TOLERANCE) {
                    state = State.MANUAL;
                }
                break;
            }
        }
    }

    public void setPosition(double setpoint) {
        state = State.PID;
        this.setPoint = setpoint;
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
