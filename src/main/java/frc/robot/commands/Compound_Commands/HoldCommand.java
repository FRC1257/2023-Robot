package frc.robot.commands.Compound_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorPIDCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import static frc.robot.Constants.PivotArm.PIVOT_ARM_SETPOINT_HOLD;
import frc.robot.subsystems.*;
import static frc.robot.Constants.ElevatorConstants.*;

public class HoldCommand extends ParallelCommandGroup{
    public HoldCommand(Elevator elevator, PivotArm pivotarm) {
        addCommands( //WITH INTAKE
            //new ElevatorExtendCommand(elevator),
            new ElevatorPIDCommand(elevator, -ELEVATOR_SETPOINT_RETRACT),
            new PivotArmPIDCommand(pivotarm, PIVOT_ARM_SETPOINT_HOLD)
            
        );

        addRequirements(elevator, pivotarm);
    }

 
}