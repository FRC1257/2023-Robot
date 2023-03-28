package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ElectricalLayout;
import frc.robot.Constants.IntakeSpeed;

public class Intake extends SnailSubsystem {

    private CANSparkMax intakeMotorRight;
    private CANSparkMax intakeMotorLeft;

    public Intake() {
        intakeMotorRight = new CANSparkMax(ElectricalLayout.INTAKE_MOTOR_LEFT_ID, MotorType.kBrushless);
        intakeMotorRight.restoreFactoryDefaults();
        intakeMotorRight.setIdleMode(IdleMode.kBrake);
        intakeMotorRight.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        intakeMotorRight.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

        intakeMotorLeft = new CANSparkMax(ElectricalLayout.INTAKE_MOTOR_RIGHT_ID, MotorType.kBrushless);
        intakeMotorLeft.restoreFactoryDefaults();
        intakeMotorLeft.setIdleMode(IdleMode.kBrake);
        intakeMotorLeft.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        intakeMotorLeft.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        intakeMotorLeft.setInverted(true);

        intakeMotorLeft.follow(intakeMotorRight);
    }

    public enum State {
        INTAKING,
        EJECTING,
        NEUTRAL,
        SHOOTING;
    }

    State intakeState = State.NEUTRAL;

    @Override
    public void update() {
        switch(intakeState) {
            case INTAKING:
                intakeMotorRight.set(IntakeSpeed.INTAKE_INTAKING_SPEED);
                break;
            case EJECTING:
                intakeMotorRight.set(IntakeSpeed.INTAKE_EJECTING_SPEED);
                break;
            case NEUTRAL:
                intakeMotorRight.set(IntakeSpeed.INTAKE_NEUTRAL_SPEED);
                break;
            case SHOOTING:
                intakeMotorRight.set(IntakeSpeed.INTAKE_SHOOTING_SPEED);
        }
        
    }

    public void neutral() {
        intakeState = State.NEUTRAL;
    }

    public void intaking() {
        intakeState = State.INTAKING;
    }

    public void ejecting() {
        intakeState = State.EJECTING;
    }

    public void shooting() {
        intakeState = State.SHOOTING;
    }

    public State getState() {
        return intakeState;
    }


    @Override
    public void displayShuffleboard() {
        SmartDashboard.putNumber("Intake Right Speed", intakeMotorRight.get());
        SmartDashboard.putNumber("Intake Left Speed", intakeMotorLeft.get());
    }

    @Override
    public void tuningInit() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void tuningPeriodic() {
        // TODO Auto-generated method stub
    
    }


} 
