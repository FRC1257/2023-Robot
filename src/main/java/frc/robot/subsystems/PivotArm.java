package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.TunableNumber;

import static frc.robot.Constants.PivotArm.*;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.NEO_550_CURRENT_LIMIT;

public class PivotArm extends SnailSubsystem {
    private CANSparkMax armMotor;
    private RelativeEncoder armEncoder;
    private RelativeEncoder betterEncoder;
    private State state = State.MANUAL;
    private double speed;
    private SparkMaxPIDController armPIDController;
    private double setPoint;

    private TunableNumber p = new TunableNumber("PivotArm", "P", PIVOT_ARM_PID[0]);
    private TunableNumber i = new TunableNumber("PivotArm", "I", PIVOT_ARM_PID[1]);
    private TunableNumber d = new TunableNumber("PivotArm", "D", PIVOT_ARM_PID[2]);
    private TunableNumber ff = new TunableNumber("PivotArm", "FF", PIVOT_ARM_PID[3]);
    private TunableNumber maxOutput = new TunableNumber("PivotArm", "Max Output", PIVOT_ARM_PID_MAX_OUTPUT);

    public enum State {
        MANUAL,
        PID
    }

    public PivotArm() {
        armMotor = new CANSparkMax(PIVOT_ARM_ID, MotorType.kBrushless);
        armMotor.restoreFactoryDefaults();
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);

        armEncoder = armMotor.getEncoder();
        armEncoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
        armEncoder.setVelocityConversionFactor(POSITION_CONVERSION_FACTOR / 60);

        betterEncoder = armMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
        betterEncoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
        betterEncoder.setVelocityConversionFactor(POSITION_CONVERSION_FACTOR / 60);

        armPIDController = armMotor.getPIDController();
        armPIDController.setP(p.get());
        armPIDController.setI(i.get());
        armPIDController.setD(d.get());
        armPIDController.setFF(ff.get());
        armPIDController.setOutputRange(-maxOutput.get(), maxOutput.get());

    }

    @Override
    public void update() {
        if ((armEncoder.getPosition() >= PIVOT_ARM_SETPOINT_TOP && speed > 0.0) 
            || (armEncoder.getPosition() <= PIVOT_ARM_SETPOINT_BOTTOM && speed < 0.0)) {
            armMotor.set(0);
            return;
        }
        
        switch (state) {
            case MANUAL:
                armMotor.set(speed);
                break;
            case PID:
                // send the desired setpoint to the PID controller and specify we want to use position control
                armPIDController.setReference(setPoint, ControlType.kPosition);

                // check our error and update the state if we finish
                if(Math.abs(armEncoder.getPosition() - setPoint) < PIVOT_ARM_PID_TOLERANCE) {
                    endPID();
                    armMotor.set(0);
                }
                break;
        }
    }

    public void setPosition(double setpoint) {
        state = State.PID;
        this.setPoint = setpoint;
    }

    // End PID
    public void endPID() {
        state = State.MANUAL;
    }
    

    @Override
    public void displayShuffleboard() {
        SmartDashboard.putBoolean("/PivotArm/Pivot Arm Bottom", armEncoder.getPosition() <= PIVOT_ARM_SETPOINT_BOTTOM /*&& speed < 0.0*/);
        SmartDashboard.putBoolean("/PivotArm/Pivot Arm Extend", armEncoder.getPosition() >= PIVOT_ARM_SETPOINT_TOP /*&& speed > 0.0*/);

        SmartDashboard.putNumber("/PivotArm/Motor Speed", armMotor.get());
        SmartDashboard.putNumber("/PivotArm/Encoder Position", armEncoder.getPosition());
        SmartDashboard.putNumber("/PivotArm/Better Encoder Position", betterEncoder.getPosition());
        SmartDashboard.putNumber("/PivotArm/Setpoint", setPoint);
        SmartDashboard.putString("/PivotArm/State", state.name());
    }

    @Override
    public void tuningInit() {
        p.reset();
        i.reset();
        d.reset();
        ff.reset();
        maxOutput.reset();
    }

    @Override 
    public void tuningPeriodic() {
        p.updateFunction(() -> armPIDController.setP(p.get()));
        i.updateFunction(() -> armPIDController.setI(i.get()));
        d.updateFunction(() -> armPIDController.setD(d.get()));
        ff.updateFunction(() -> armPIDController.setFF(ff.get()));
        maxOutput.updateFunction(() -> armPIDController.setOutputRange(-maxOutput.get(), maxOutput.get()));

        if (SmartDashboard.getBoolean("Motor mode", false) && armMotor.getIdleMode() != IdleMode.kBrake) {
            armMotor.setIdleMode(IdleMode.kBrake);
        } else if (!SmartDashboard.getBoolean("Motor mode", false) && armMotor.getIdleMode() != IdleMode.kCoast) {
            armMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    public void manualControl(double newSpeed) {
        speed = newSpeed;
        state = State.MANUAL;
    }

    public State getState() { return state; }

    public boolean atSetpoint() {
        return Math.abs(armEncoder.getPosition() - setPoint) < PIVOT_ARM_PID_TOLERANCE;
    }
}
