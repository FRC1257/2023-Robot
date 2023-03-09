package frc.robot.commands.elevator;

import frc.robot.subsystems.Elevator.Elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorManualCommand extends CommandBase{
    private Elevator elevator;
    private double speed;
	public ElevatorManualCommand(Elevator elevator, double speed) {
        this.elevator = elevator;
        this.speed = speed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        elevator.manual(speed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}