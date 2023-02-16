package frc.robot.commands.pivotArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotArm;

public class PivotArmManualCommand extends CommandBase{
	private PivotArm pivotArm;
    private double speed;
	public PivotArmManualCommand(PivotArm pivotArm, double speed) {
        this.pivotArm = pivotArm;
this.speed = speed;
        addRequirements(pivotArm);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        pivotArm.manualControl(speed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}