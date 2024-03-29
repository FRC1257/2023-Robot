package frc.robot.commands.Compound_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.PivotArm.PIVOT_ARM_SETPOINT_UP;

import frc.robot.commands.elevator.ElevatorPIDCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotArm;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_SETPOINT_EXTEND;

public class MidConeSetpointCommand extends SequentialCommandGroup{
    
    public MidConeSetpointCommand(Elevator elevator, PivotArm pivotarm) {
        // TODO: Add new command sequence given Cube Shooting 
        addCommands(
            new ParallelCommandGroup(new ElevatorPIDCommand(elevator, -ELEVATOR_SETPOINT_EXTEND),
            new PivotArmPIDCommand(pivotarm, PIVOT_ARM_SETPOINT_UP))
            
        );

        addRequirements(elevator, pivotarm);
    }
    
    
}