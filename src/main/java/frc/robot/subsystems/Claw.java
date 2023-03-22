package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.Claw.*;
import static frc.robot.Constants.NEO_550_CURRENT_LIMIT;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElectricalLayout;
import frc.robot.util.TunableNumber;

public class Claw extends SnailSubsystem {
    private CANSparkMax motorLeft;
    private CANSparkMax motorRight;
    // private DoubleSolenoid solenoid;
    // private Compressor compressor;

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
    
    //cube and cone states
    private RollerState rollerState;
    
    public Claw() {
        motorLeft = new CANSparkMax(ElectricalLayout.CLAW_MOTOR_LEFT_ID, MotorType.kBrushless);
        motorRight = new CANSparkMax(ElectricalLayout.CLAW_MOTOR_RIGHT_ID, MotorType.kBrushless);        

        motorLeft.restoreFactoryDefaults();
        motorLeft.setIdleMode(IdleMode.kBrake);
        motorLeft.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);

        motorRight.restoreFactoryDefaults();
        motorRight.setIdleMode(IdleMode.kBrake);
        motorRight.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
        // motorInit(motorLeft);
        // motorInit(motorLeft);

        // motorRight.follow(motorLeft, true);
        rollerState = RollerState.NEUTRAL;
    }

    private void motorInit(CANSparkMax motor) {
        
    }
    
    @Override
    public void update() {
        switch(rollerState) {
            case NEUTRAL:
                motorLeft.set(neutralSpeed.get());
                motorRight.set(-(neutralSpeed.get()));
                break;
            case INTAKING:
                motorLeft.set(intakeSpeed.get());
                motorRight.set(-(intakeSpeed.get()));
                break;
            case SHOOTING:
                motorLeft.set(shootingSpeed.get());
                motorRight.set(-shootingSpeed.get());
                break;
            case EJECTING:
                motorLeft.set(ejectSpeed.get());
                motorRight.set(-ejectSpeed.get());
                break;
        }
    }

    @Override
    public void displayShuffleboard() {
        SmartDashboard.putNumber("Claw Left Motor Speed", motorLeft.get());
        SmartDashboard.putNumber("Claw Right Motor Speed", motorRight.get());
        SmartDashboard.putString("Claw Roller State", rollerState.toString());
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
    
    public RollerState getRollerState() {
        return rollerState;
    }
}
