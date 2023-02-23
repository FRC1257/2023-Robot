package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElectricalLayout;
import frc.robot.Constants;
import frc.robot.Constants.IntakeSpeed;

public class Intake extends SnailSubsystem {

    private CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(ElectricalLayout.INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        intakeMotor.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
    }

    public enum State {
        INTAKING,
        EJECTING,
        NEUTRAL;
    }

    State intakeState = State.NEUTRAL;

    @Override
    public void update() {
        switch(intakeState) {
            case INTAKING:
                intakeMotor.set(IntakeSpeed.INTAKE_INTAKING_SPEED);
                break;
            case EJECTING:
                intakeMotor.set(IntakeSpeed.INTAKE_EJECTING_SPEED);
                break;
            case NEUTRAL:
                intakeMotor.set(IntakeSpeed.INTAKE_NEUTRAL_SPEED);
                break;
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

    public State getState() {
        return intakeState;
    }


    @Override
    public void displayShuffleboard() {
        SmartDashboard.putNumber("Intake Speed", intakeMotor.get());
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

