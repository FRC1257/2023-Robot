package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawCubeStateCommand extends CommandBase {
	private Claw claw;

	public ClawCubeStateCommand (Claw claw) {
		this.claw = claw;
		addRequirements(claw);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		claw.cubeintake();
	}

	@Override 
	public void end(boolean interrupted) {

	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
