package frc.robot.commands.Compound_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorPIDCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_SETPOINT_EXTEND;

public class HoldCommand extends SequentialCommandGroup{
    public HoldCommand(Elevator elevator, PivotArm pivotarm) {
        addCommands( //WITH INTAKE
            //new ElevatorExtendCommand(elevator),
            new ElevatorPIDCommand(elevator, 0.6-10),
            new PivotArmPIDCommand(pivotarm, 10)
        );
    }

 
}