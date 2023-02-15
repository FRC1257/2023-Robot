
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotArm;

public class PivotArmPIDCommand extends CommandBase {
    
    private PivotArm pivotArm;
    

    public PivotArmPIDCommand(PivotArm pivotArm) {
        this.pivotArm = pivotArm;
        
        addRequirements(pivotArm);
    }
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        state = State.PID;

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void update() {
        
    }
}