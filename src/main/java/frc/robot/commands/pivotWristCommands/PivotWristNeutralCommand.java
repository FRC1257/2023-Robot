// need to bind command
package frc.robot.commands.pivotWristCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotWrist;

public class PivotWristNeutralCommand extends CommandBase {
    private PivotWrist pivotWrist;

    public PivotWristNeutralCommand(PivotWrist pivotWrist){
    this.pivotWrist = pivotWrist;
    addRequirements(pivotWrist);
    
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
    pivotWrist.neutral();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
    return false;
    }
}