package frc.robot.commands.Compound_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.PivotArm.PIVOT_ARM_SETPOINT_MID;


import frc.robot.commands.elevator.ElevatorPIDCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotArm;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_SETPOINT_EXTEND;


public class MidCubeSetpointCommand extends SequentialCommandGroup {
    
    // TODO: Add new command sequence based off of Cube Shooting
    public MidCubeSetpointCommand(Elevator elevator, PivotArm pivotarm) {
        addCommands(
            new ParallelCommandGroup(new ElevatorPIDCommand(elevator, -ELEVATOR_SETPOINT_EXTEND), 
            new PivotArmPIDCommand(pivotarm, PIVOT_ARM_SETPOINT_MID))
            
        );

        addRequirements(elevator, pivotarm);
    }
}