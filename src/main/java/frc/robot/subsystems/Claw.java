package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.Claw.*;
import static frc.robot.Constants.NEO_550_CURRENT_LIMIT;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElectricalLayout;
import frc.robot.util.TunableNumber;

public class Claw extends SnailSubsystem {
    private CANSparkMax clawMotor;
    private RelativeEncoder clawEncoder;
    private double speed;
    private double setPoint;

    private double addSpeed = 0;
    
    public enum ClawMoveState {
        OPENING,
        CLOSING
    }

    public enum ClawState {
      MANUAL
    }
    
    private ClawState clawState;
    private ClawMoveState clawMoveState;
    private TunableNumber deez = new TunableNumber("Claw", "Claw Motor Close Speed", 0);
    
    public Claw() {
        clawMotor = new CANSparkMax(ElectricalLayout.CLAW_MOTOR_LEFT_ID, MotorType.kBrushless);
        clawMotor.restoreFactoryDefaults();
        clawMotor.setIdleMode(IdleMode.kBrake);
        clawMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
        clawMotor.setInverted(true);

        clawEncoder = clawMotor.getEncoder();
        clawEncoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
        clawEncoder.setVelocityConversionFactor(POSITION_CONVERSION_FACTOR / 60);

        clawState = ClawState.MANUAL;
        clawMoveState = ClawMoveState.CLOSING;
        
    }
    
    @Override
    public void update() {

        switch(clawState) {
            case MANUAL:
                 if (speed > 0.1) {
                    clawMoveState = ClawMoveState.OPENING;
                } else if (speed < -0.1) {
                    clawMoveState = ClawMoveState.CLOSING;
                } 
                clawMotor.set(speed + addSpeed);
                break;
        } 

        switch (clawMoveState) {
            case OPENING:
                addSpeed = -0.01;
                break;
            case CLOSING:
                addSpeed = deez.get();
                break;
        } 
    }

    public void endPID() {
        this.clawState = ClawState.MANUAL;
    }

    @Override
    public void displayShuffleboard() {
        SmartDashboard.putNumber("/Claw/Claw Motor Speed", clawMotor.get());
        SmartDashboard.putNumber("/Claw/Claw Encoder Position", clawEncoder.getPosition());
        SmartDashboard.putNumber("/Claw/Claw Setpoint", setPoint);
        SmartDashboard.putString("/Claw/Claw Motion State", clawState.name());
        SmartDashboard.putString("/Claw/Claw State", clawMoveState.name());
    }

    @Override
    public void tuningInit() {}

    @Override
    public void tuningPeriodic() {}

    public void manualControl(double newSpeed) {
        clawState = ClawState.MANUAL;
        speed = newSpeed;
    }

    public ClawState getMotionState() {
        return clawState;
    }

    public ClawMoveState getState() {
        return clawMoveState;
    }
}
