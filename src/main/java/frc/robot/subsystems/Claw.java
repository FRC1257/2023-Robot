package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.Claw.*;
import static frc.robot.Constants.NEO_CURRENT_LIMIT;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private RollerState rollerState;
    private ClawState clawState;
    
    public Claw() {
        motorLeft = new CANSparkMax(CLAW_MOTOR_LEFT_ID, MotorType.kBrushless);
        motorRight = new CANSparkMax(CLAW_MOTOR_RIGHT_ID, MotorType.kBrushless);
        motorInit(motorLeft);
        motorInit(motorRight);
        motorRight.follow(motorLeft, true);
        rollerState = RollerState.NEUTRAL;
        
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, CLAW_FORWARD_ID, CLAW_REVERSE_ID);
        clawState = ClawState.CUBEINTAKE;
    }
    private void motorInit(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
    }
    @Override
    public void update() {
        switch(rollerState) {
            case NEUTRAL:
                motorLeft.set(ROLLER_NEUTRAL_SPEED);
                break;
            case INTAKING:
                motorLeft.set(ROLLER_INTAKING_SPEED);
                break;
            case EJECTING:
                motorLeft.set(ROLLER_EJECTING_SPEED);
                break;
        }
        
        switch(clawState) {
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
        rollerState = RollerState.NEUTRAL;
    }

    public void eject() {
        rollerState = RollerState.EJECTING;
    }

    public void intake() {
        rollerState = RollerState.INTAKING;
    }
    
    public void cubeintake() {
        clawState = ClawState.CUBEINTAKE;
    }
    
    public void coneintake() {
        clawState = ClawState.CONEINTAKE;
    }
    
    public RollerState getRollerState() {
        return rollerState;
    }
    public ClawState getClawState() {
        return clawState;
    }
}
