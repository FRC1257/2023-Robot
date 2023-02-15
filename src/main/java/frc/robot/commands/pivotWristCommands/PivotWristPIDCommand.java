// need to bind command
package frc.robot.commands.rollerintake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotWrist;

public class PivotWristPIDCommand extends CommandBase {
    private PivotWrist pivotWrist;

    public PivotWristPIDCommand(PivotWrist pivotWrist)
    this.pivotWrist = pivotWrist;
    addRequirements(pivotWrist);
    
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
    pivotWrist.setPosition();
    }

    @Override
    public void end(boolean interrupted) {
    pivotWrist.endPID();
    }

    @Override
    public boolean isFinished() {
    return pivotWrist.getState() != pivotWrist.State.PID;
    }

