
package frc.robot.commands.pivotArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotArm;

public class PivotArmManualCommand extends CommandBase {
    
    private PivotArm pivotArm;
    

    public PivotArmManualCommand(PivotArm pivotArm) {
        this.pivotArm = pivotArm;
        
        addRequirements(pivotArm);
    }
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}