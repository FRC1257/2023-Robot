package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;


import edu.wpi.first.wpilibj.DoubleSolenoid;
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
        motorLeft = new CANSparkMax(CLAW_MOTOR_LEFT, MotorType.kBrushless);
        motorRight = new CANSparkMax(CLAW_MOTOR_RIGHT, MotorType.kBrushless);
        motorInit(motorLeft);
        motorInit(motorRight);
        motorRight.follow(motorLeft, true);
        rollerState = RollerState.NEUTRAL;
        
        solenoid = new DoubleSolenoid(CLAW_FORWARD_ID, CLAW_REVERSE_ID);
        clawState = ClawState.CUBEINTAKE;
    }
    private void motorInit(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(Constants.NEO_550_CURRENT_LIMIT);
    }
    @Override
    public void update() {
        switch(rollerState) {
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
