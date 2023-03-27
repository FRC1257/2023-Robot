package frc.robot.commands.IntakeArm;

import edu.wpi.first.wpilibj.RobotBase;
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
        if (RobotBase.isSimulation()) {
            return true;
        }
        return intakearm.getState() != IntakeArm.State.PID;
    }
}