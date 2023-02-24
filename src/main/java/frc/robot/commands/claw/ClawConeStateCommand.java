package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawConeStateCommand extends CommandBase {
	private Claw claw;

	public ClawConeStateCommand (Claw claw) {
		this.claw = claw;
		addRequirements(claw);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		claw.coneintake();
	}

	@Override 
	public void end(boolean interrupted) {

	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
