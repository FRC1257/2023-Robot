package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawAlt extends SnailSubsystem {

    public enum State {
      INTAKING,
      EJECTING,
      NEUTRAL
    }

    public ClawAlt() {

    }

    @Override
    public void update() {

    }

    @Override
    public void displayShuffleboard() {

    }

    @Override
    public void tuningInit() {

    }

    @Override
    public void tuningPeriodic() {

    }

    public State getState() {
        return state;
    }
}
