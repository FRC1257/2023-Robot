package frc.robot.commands.pivotArm;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotArm;

public class PivotArmPIDCommand extends CommandBase {
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
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        pivotArm.endPID();
    }

    @Override
    public boolean isFinished() {
        if (RobotBase.isSimulation())
            return true;
        return pivotArm.getState() != PivotArm.State.PID;
    }
}