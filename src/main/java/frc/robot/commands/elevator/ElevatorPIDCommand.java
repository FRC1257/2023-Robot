package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorPIDCommand extends CommandBase{
    private Elevator elevator;
	private double setpoint;
	
	public ElevatorPIDCommand(Elevator elevator, double setpoint) {
		this.elevator = elevator;
		this.setpoint = setpoint;
        addRequirements(elevator);
	}

	@Override
    public void initialize() {
        elevator.setPosition(setpoint);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        // elevator.endPID();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
