// need to bind command
package frc.robot.commands.pivotWrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotWrist.PivotWrist;
import frc.robot.subsystems.PivotWrist.PivotWristIO;

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
        pivotWrist.manualControl(0);
    }

    @Override
    public boolean isFinished() {
        return pivotWrist.getState() != PivotWristIO.State.PID;
    }
}
