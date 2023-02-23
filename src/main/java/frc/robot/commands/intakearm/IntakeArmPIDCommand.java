package frc.robot.commands.intakearm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArm;

public class IntakeArmPIDCommand extends CommandBase{
	private IntakeArm intakearm;
	private double setpoint;
	
	public IntakeArmPIDCommand(IntakeArm intakearm, double setpoint) {
		this.intakearm = intakearm;
		this.setpoint = setpoint;
        addRequirements(intakearm);
	}

	@Override
    public void initialize() {
        intakearm.setPosition(setpoint);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        intakearm.endPID();
    }

    @Override
    public boolean isFinished() {
        return intakearm.getState() != IntakeArm.State.PID;
    }
}
