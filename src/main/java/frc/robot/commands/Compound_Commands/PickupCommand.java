package frc.robot.commands.Compound_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.elevator.ElevatorPIDCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotArm;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_SETPOINT_EXTEND;


public class PickupCommand extends ParallelCommandGroup {
    
    // TODO: Add new command sequence based off of Cube Shooting
    // erick prefered
    public PickupCommand(Elevator elevator, PivotArm pivotarm) {
        addCommands(
            new ParallelCommandGroup(new ElevatorPIDCommand(elevator, -ELEVATOR_SETPOINT_EXTEND), 
            new PivotArmPIDCommand(pivotarm, 110))
            
        );

        addRequirements(elevator, pivotarm);
    }
}