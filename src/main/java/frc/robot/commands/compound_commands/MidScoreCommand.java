package frc.robot.commands.compound_commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.PivotArm.PIVOT_ARM_SETPOINT_MID;
import static frc.robot.Constants.PivotWrist.WRIST_SETPOINT_MID;
import frc.robot.commands.elevator.ElevatorExtendCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.commands.pivotWrist.PivotWristPIDCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.PivotWrist;


public class MidScoreCommand extends SequentialCommandGroup {
    
    // before command: claw has game piece grabbed from intake
    public MidScoreCommand(Elevator elevator, PivotArm pivotarm, PivotWrist pivotWrist) {
        addCommands(
            new ElevatorExtendCommand(elevator), 
            new ParallelCommandGroup(
                new PivotArmPIDCommand(pivotarm, PIVOT_ARM_SETPOINT_MID), 
                new PivotWristPIDCommand(pivotWrist, WRIST_SETPOINT_MID)
            )
        );
    }
}
