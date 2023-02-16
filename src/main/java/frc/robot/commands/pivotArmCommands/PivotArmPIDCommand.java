package frc.robot.commands.pivotArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotArm;

public class PivotArmPIDCommand extends CommandBase{
	private PivotArm pivotArm;
	private double setPoint;
	
	public PivotArmPIDCommand(PivotArm pivotArm, double setPoint) {
		this.pivotArm = pivotArm;
		this.setPoint = setPoint;
        addRequirements(pivotArm);
	}

	@Override
    public void initialize() {
        pivotArm.setPosition(setPoint);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        pivotArm.manualControl(0);
    }

    @Override
    public boolean isFinished() {
        return pivotArm.getState() != PivotArm.State.PID;
    }
}