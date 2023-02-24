package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawIntakeCommand extends CommandBase {
	private Claw claw;

	public ClawIntakeCommand(Claw claw) {
		this.claw = claw;
		addRequirements(claw);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		claw.intake();
	}

	@Override 
	public void end(boolean interrupted) {

	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
