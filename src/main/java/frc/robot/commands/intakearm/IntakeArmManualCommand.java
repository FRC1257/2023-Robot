package frc.robot.commands.intakearm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArm;

public class IntakeArmManualCommand extends CommandBase{
	private IntakeArm intakearm;
    private double speed;
	public IntakeArmManualCommand(IntakeArm intakearm, double speed) {
        this.intakearm = intakearm;
        this.speed = speed;
        addRequirements(intakearm);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        intakearm.manual(speed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
