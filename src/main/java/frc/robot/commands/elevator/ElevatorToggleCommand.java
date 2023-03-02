package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorToggleCommand extends CommandBase {

    private Elevator elevator;

    public ElevatorToggleCommand(Elevator elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(elevator.getState() == Elevator.State.EXTENDED) {
            elevator.retract();
        } else {
            elevator.extended();
        }
    }


    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
