package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.Claw.*;
import static frc.robot.Constants.NEO_550_CURRENT_LIMIT;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElectricalLayout;
import frc.robot.util.TunableNumber;

public class Claw extends SnailSubsystem {
    private TunableNumber p = new TunableNumber("Pivot Arm P", CLAW_PID[0]);
    private TunableNumber i = new TunableNumber("Pivot Arm I", CLAW_PID[1]);
    private TunableNumber d = new TunableNumber("Pivot Arm D", CLAW_PID[2]);
    private TunableNumber ff = new TunableNumber("Pivot Arm FF", CLAW_PID[3]);
    private TunableNumber maxOutput = new TunableNumber("Pivot Arm Max Output", CLAW_PID_MAX_OUTPUT);

    private CANSparkMax clawMotor;
    private RelativeEncoder clawEncoder;
    private SparkMaxPIDController clawPIDController;
    private double speed;
    private double setPoint;
    
    public enum ClawState {
        OPEN,
        CLOSED
    }

    public enum ClawMotionState {
      MANUAL,
      PID
    }
    
    private ClawMotionState clawMotionState;
    private ClawState clawState;
    
    public Claw() {
        clawMotor = new CANSparkMax(ElectricalLayout.CLAW_MOTOR_LEFT_ID, MotorType.kBrushless);
        clawMotor.restoreFactoryDefaults();
        clawMotor.setIdleMode(IdleMode.kBrake);
        clawMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);

        clawEncoder = clawMotor.getEncoder();
        clawEncoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
        clawEncoder.setVelocityConversionFactor(POSITION_CONVERSION_FACTOR / 60);

        clawPIDController = clawMotor.getPIDController();
        clawPIDController.setP(p.get());
        clawPIDController.setI(i.get());
        clawPIDController.setD(d.get());
        clawPIDController.setFF(ff.get());
        clawPIDController.setOutputRange(-maxOutput.get(), maxOutput.get());

        clawMotionState = ClawMotionState.MANUAL;
    }
    
    @Override
    public void update() {
        SmartDashboard.putBoolean("Claw Closed", clawEncoder.getPosition() <= CLAW_SETPOINT_CLOSED /*&& speed < 0.0*/);
        SmartDashboard.putBoolean("Claw Open", clawEncoder.getPosition() >= CLAW_SETPOINT_OPEN /*&& speed > 0.0*/);

        switch(clawMotionState) {
            case MANUAL:
                clawMotor.set(speed);
                break;
            case PID:
                clawPIDController.setReference(setPoint, ControlType.kPosition);
                break;
        }
    }

    public void setPosition(double setpoint) {
        this.clawMotionState = ClawMotionState.PID;
        this.setPoint = setpoint;
    }

    public void close() {
        setPosition(CLAW_SETPOINT_CLOSED);
        clawState = ClawState.CLOSED;
    }

    public void open() {
        setPosition(CLAW_SETPOINT_OPEN);
    }

    public void endPID() {
        this.clawMotionState = ClawMotionState.MANUAL;
    }

    @Override
    public void displayShuffleboard() {
        SmartDashboard.putNumber("Claw Motor Speed", clawMotor.get());
        SmartDashboard.putNumber("Claw Encoder Position", clawEncoder.getPosition());
        SmartDashboard.putNumber("Claw Setpoint", setPoint);
        SmartDashboard.putString("Claw State", clawMotionState.name());
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
        p.updateFunction(() -> clawPIDController.setP(p.get()));
        i.updateFunction(() -> clawPIDController.setI(i.get()));
        d.updateFunction(() -> clawPIDController.setD(d.get()));
        ff.updateFunction(() -> clawPIDController.setFF(ff.get()));
        maxOutput.updateFunction(() -> clawPIDController.setOutputRange(-maxOutput.get(), maxOutput.get()));
    }

    public void manualControl(double newSpeed) {
        clawMotionState = ClawMotionState.MANUAL;
        speed = newSpeed;
    }

    public boolean atSetpoint() {
        return Math.abs(clawEncoder.getPosition() - setPoint) < CLAW_PID_TOLERANCE;
    }
    
    public ClawMotionState getMotionState() {
        return clawMotionState;
    }

    public ClawState getState() {
        return clawState;
    }
}
