package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.OtherCoolController;
import frc.robot.util.TunableNumber;

import static frc.robot.Constants.PivotArm.*;
import static frc.robot.Constants.*;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.NEO_550_CURRENT_LIMIT;

public class PivotArm extends SnailSubsystem {
    private CANSparkMax armMotor;
    private RelativeEncoder armEncoder;
    private DutyCycleEncoder absoluteEncoder;
    private State state = State.MANUAL_PID;
    private double position;
    private SparkMaxPIDController armPIDController;
    private double setPoint;

    private TunableNumber p = new TunableNumber("PivotArmValues", "P", PIVOT_ARM_PID[0]);
    private TunableNumber i = new TunableNumber("PivotArmValues", "I", PIVOT_ARM_PID[1]);
    private TunableNumber d = new TunableNumber("PivotArmValues", "D", PIVOT_ARM_PID[2]);
    private TunableNumber ff = new TunableNumber("PivotArmValues", "FF", PIVOT_ARM_PID[3]);
    private TunableNumber maxOutput = new TunableNumber("PivotArmValues", "Max Output", PIVOT_ARM_PID_MAX_OUTPUT);

    private Timer temp_timer = new Timer();

    public enum State {
        MANUAL_PID,
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
        

        absoluteEncoder = new DutyCycleEncoder(1);
        absoluteEncoder.setDistancePerRotation(360.0 / 1024.0);
        absoluteEncoder.setDutyCycleRange(1/1024.0, 1023.0/1024.0);

        armPIDController = armMotor.getPIDController();
        armPIDController.setP(p.get());
        armPIDController.setI(i.get());
        armPIDController.setD(d.get());
        armPIDController.setFF(ff.get());
        armPIDController.setOutputRange(-maxOutput.get(), maxOutput.get());

        armEncoder.setPosition((absoluteEncoder.getAbsolutePosition() - 0.475) * 285);
        
    }

    @Override
    public void update() {
        if (armMotor.getMotorTemperature() > HIGH_TEMP) {
            if (temp_timer.get() == 0.0) {
                temp_timer.start();
            } else if (temp_timer.get() > HIGH_TEMP_TIME) {
                // swap out for alert later
                armMotor.set(0.0);
                return;
            }
        } else {
            temp_timer.stop();
            temp_timer.reset();
        }
        
        switch (state) {
            case MANUAL_PID:
                armPIDController.setReference(position, ControlType.kPosition);
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
        state = State.MANUAL_PID;
    }
    

    @Override
    public void displayShuffleboard() {
        SmartDashboard.putBoolean("PivotArm Pivot Arm Bottom", armEncoder.getPosition() <= PIVOT_ARM_SETPOINT_BOTTOM /*&& position < 0.0*/);
        SmartDashboard.putBoolean("PivotArm Pivot Arm Extend", armEncoder.getPosition() >= PIVOT_ARM_SETPOINT_TOP /*&& position > 0.0*/);

        SmartDashboard.putNumber("PivotArm Motor position", armMotor.get());
        SmartDashboard.putNumber("PivotArm Encoder Position", armEncoder.getPosition());
        SmartDashboard.putNumber("PivotArm Better Encoder Position", absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("PivotArm Setpoint", setPoint);
        SmartDashboard.putString("PivotArm State", state.name());
        SmartDashboard.putNumber("PivotArm Motor Temp", armMotor.getMotorTemperature());
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

    public void manual_PIDControl(double position) {
        position = OtherCoolController.mapRange(position, -1, 1, PIVOT_ARM_SETPOINT_BOTTOM, PIVOT_ARM_SETPOINT_TOP);
        state = State.MANUAL_PID;
    }

    public State getState() { return state; }

    public boolean atSetpoint() {
        return Math.abs(armEncoder.getPosition() - setPoint) < PIVOT_ARM_PID_TOLERANCE;
    }
}