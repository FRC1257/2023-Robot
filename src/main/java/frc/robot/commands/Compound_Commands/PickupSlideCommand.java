package frc.robot.commands.Compound_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.elevator.ElevatorPIDCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotArm;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_SETPOINT_RETRACT;
import static frc.robot.Constants.PivotArm.PIVOT_ARM_SETPOINT_SLIDE;

public class PickupSlideCommand extends ParallelCommandGroup {
    
    // TODO: Add new command sequence based off of Cube Shooting
    // erick prefered
    public PickupSlideCommand(Elevator elevator, PivotArm pivotarm) {
        addCommands(
            new ParallelCommandGroup(new ElevatorPIDCommand(elevator, -ELEVATOR_SETPOINT_RETRACT), 
            new PivotArmPIDCommand(pivotarm, PIVOT_ARM_SETPOINT_SLIDE))
        );

        addRequirements(elevator, pivotarm);
    }
}