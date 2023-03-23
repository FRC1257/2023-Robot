package frc.robot.commands.Compound_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorPIDCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_SETPOINT_RETRACT;

public class IntakeCommand extends SequentialCommandGroup{
    public IntakeCommand(Elevator elevator, PivotArm pivotarm) {
        addCommands( //WITH INTAKE
            //new ElevatorExtendCommand(elevator),
            new ParallelCommandGroup( // run intake entire time
                new ElevatorPIDCommand(elevator, ELEVATOR_SETPOINT_RETRACT),
                new SequentialCommandGroup(
                    new PivotArmPIDCommand(pivotarm, Constants.PivotArm.PIVOT_ARM_SETPOINT_INTAKE)
                )   
            )
        );
    }

 
}