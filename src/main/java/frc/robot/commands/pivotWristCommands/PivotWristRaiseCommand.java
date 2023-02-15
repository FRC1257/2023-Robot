// need to bind command
package frc.robot.commands.rollerintake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotWrist;

public class PivotWristRaiseCommand extends CommandBase {
    private PivotWrist pivotWrist;

    public PivotWristRaiseCommand(PivotWrist pivotWrist)
    this.pivotWrist = pivotWrist;
    addRequirements(pivotWrist);
    
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
    pivotWrist.raise();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
    return false;
    }