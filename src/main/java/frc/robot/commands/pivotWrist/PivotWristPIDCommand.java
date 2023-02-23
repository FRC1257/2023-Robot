// need to bind command
package frc.robot.commands.pivotWrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotWrist;

public class PivotWristPIDCommand extends CommandBase {
    private PivotWrist pivotWrist;
    private double setPoint;

    public PivotWristPIDCommand(PivotWrist pivotWrist, double setPoint) {
        this.pivotWrist = pivotWrist;
        this.setPoint = setPoint;
        addRequirements(pivotWrist);
    }

    @Override
    public void initialize() {
        pivotWrist.setPosition(setPoint);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        pivotWrist.endPID();
    }

    @Override
    public boolean isFinished() {
        return pivotWrist.getState() != PivotWrist.State.PID;
    }
}
