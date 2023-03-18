package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawItemToggleCommand extends CommandBase {
	private Claw claw;

	public ClawItemToggleCommand (Claw claw) {
		this.claw = claw;
		addRequirements(claw);
	}

	@Override
	public void initialize() {
		if(claw.getClawState() == Claw.ClawState.CUBEINTAKE) {
			claw.coneintake();
		} else {
			claw.cubeintake();
		}
	}

	@Override
	public void execute() {
		
	}

	@Override 
	public void end(boolean interrupted) {

	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
