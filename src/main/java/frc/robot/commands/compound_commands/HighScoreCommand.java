package frc.robot.commands.compound_commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.PivotWrist.WRIST_SETPOINT_TOP;
import static frc.robot.Constants.PivotArm.PIVOT_ARM_SETPOINT_UP;

import frc.robot.commands.elevator.ElevatorExtendCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.commands.pivotWrist.PivotWristPIDCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.PivotWrist;

public class HighScoreCommand extends SequentialCommandGroup{
    
    public HighScoreCommand(Elevator elevator, PivotArm pivotarm, PivotWrist pivotwrist) {
        
        addCommands(
            new ElevatorExtendCommand(elevator),
            new ParallelCommandGroup(
                new PivotWristPIDCommand(pivotwrist, WRIST_SETPOINT_TOP),
                new PivotArmPIDCommand(pivotarm, PIVOT_ARM_SETPOINT_UP)
            )
        );
    }
    
}
