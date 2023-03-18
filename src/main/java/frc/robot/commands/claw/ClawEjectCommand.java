package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawEjectCommand extends CommandBase {
	private Claw claw;

	public ClawEjectCommand(Claw claw) {
		this.claw = claw;
		addRequirements(claw);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		claw.eject();
	}

	@Override 
	public void end(boolean interrupted) {
		claw.neutral();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
