package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorIdleCommand extends CommandBase{
    private Elevator elevator;

    public ElevatorIdleCommand(Elevator elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevator.idle();
    }


    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
