package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorRetractCommand extends CommandBase {

    private Elevator elevator;

    public ElevatorRetractCommand(Elevator elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevator.retract();
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}