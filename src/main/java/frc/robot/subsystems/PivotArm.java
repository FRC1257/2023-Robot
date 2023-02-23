package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private DigitalInput limitSwitch;


    public enum State {
        MANUAL,
        PID
    }

    public PivotArm() {
        leftArmMotor = new CANSparkMax(PIVOT_ARM_LEFT_ID, MotorType.kBrushless);
        leftArmMotor.restoreFactoryDefaults();
        leftArmMotor.setIdleMode(IdleMode.kBrake);
        leftArmMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);

        rightArmMotor = new CANSparkMax(PIVOT_ARM_RIGHT_ID, MotorType.kBrushless);
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

        limitSwitch = new DigitalInput(INTAKE_ARM_BUMP_SWITCH_ID);
    }

    @Override
    public void update() {
        if (limitSwitch.get()) {
            leftArmEncoder.setPosition(0);
            if (speed < 0) {
                speed = 0;
            }
        }

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
        SmartDashboard.putNumber("Motor Speed", leftArmEncoder.getVelocity());
        SmartDashboard.putNumber("Encoder Position", leftArmEncoder.getPosition());
        SmartDashboard.putNumber("Setpoint", setPoint);
        SmartDashboard.putBoolean("Limit Switch State", limitSwitch.get());
    }

    @Override
    public void tuningInit() {
        // TODO include PID tuning
        SmartDashboard.putNumber("Pivot Arm P", PIVOT_ARM_PID[0]);
        SmartDashboard.putNumber("Pivot Arm I", PIVOT_ARM_PID[1]);
        SmartDashboard.putNumber("Pivot Arm D", PIVOT_ARM_PID[2]);

        SmartDashboard.putNumber("Pivot Arm Max Output", PIVOT_ARM_PID_MAX_OUTPUT);
        SmartDashboard.putNumber("Pivot Arm Tolerance", PIVOT_ARM_PID_TOLERANCE);
    }

    @Override
    public void tuningPeriodic() {
        PIVOT_ARM_PID[0] = SmartDashboard.getNumber("Pivot Arm P", PIVOT_ARM_PID[0]);
        PIVOT_ARM_PID[1] = SmartDashboard.getNumber("Pivot Arm I", PIVOT_ARM_PID[1]);
        PIVOT_ARM_PID[2] = SmartDashboard.getNumber("Pivot Arm D", PIVOT_ARM_PID[2]);
        
        if (PIVOT_ARM_PID[0] != armPIDController.getP()) {
            armPIDController.setP(PIVOT_ARM_PID[0]);
        }
        if (PIVOT_ARM_PID[1] != armPIDController.getI()) {
            armPIDController.setI(PIVOT_ARM_PID[1]);
        }
        if (PIVOT_ARM_PID[2] != armPIDController.getD()) {
            armPIDController.setD(PIVOT_ARM_PID[2]);
        }
    }

    public void manualControl(double newSpeed) {
        speed = newSpeed;
        state = State.MANUAL;
    }

    public State getState() { return state; }
}
