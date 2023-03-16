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
import frc.robot.Constants.ElectricalLayout;
import frc.robot.util.TunableNumber;

public class Claw extends SnailSubsystem {
    private CANSparkMax motorLeft;
    private CANSparkMax motorRight;
    private DoubleSolenoid solenoid;

    private TunableNumber neutralSpeed = new TunableNumber("Claw Neutral Speed", ROLLER_NEUTRAL_SPEED);
    private TunableNumber intakeSpeed = new TunableNumber("Claw Intake Speed", ROLLER_INTAKING_SPEED);
    private TunableNumber shootingSpeed = new TunableNumber("Claw Shooting Speed", ROLLER_SHOOTING_SPEED);
    private TunableNumber ejectSpeed = new TunableNumber("Claw Eject Speed", ROLLER_EJECTING_SPEED);
    
    
    public enum RollerState {
      INTAKING,
      EJECTING,
      SHOOTING,
      NEUTRAL
    }
    
    public enum ClawState {
      CUBEINTAKE,
      CONEINTAKE
    }
    
    //cube and cone states
    private RollerState rollerState;
    private ClawState clawState;
    
    public Claw() {
        motorLeft = new CANSparkMax(ElectricalLayout.CLAW_MOTOR_LEFT_ID, MotorType.kBrushless);
        motorRight = new CANSparkMax(ElectricalLayout.CLAW_MOTOR_RIGHT_ID, MotorType.kBrushless);        

        motorInit(motorLeft);
        motorInit(motorLeft);

        motorRight.follow(motorLeft, true);
        rollerState = RollerState.NEUTRAL;
        
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ElectricalLayout.CLAW_FORWARD_ID, ElectricalLayout.CLAW_REVERSE_ID);
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
                motorLeft.set(neutralSpeed.get());
                break;
            case INTAKING:
                motorLeft.set(intakeSpeed.get());
                break;
            case SHOOTING:
                motorLeft.set(shootingSpeed.get());
                break;
            case EJECTING:
                motorLeft.set(ejectSpeed.get());
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
        SmartDashboard.putNumber("Left Motor", motorLeft.get());
        SmartDashboard.putNumber("Right Motor", motorRight.get());
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

    public void shoot() {
        rollerState = RollerState.SHOOTING;
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
