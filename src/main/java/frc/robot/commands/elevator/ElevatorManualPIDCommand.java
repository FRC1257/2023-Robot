package frc.robot.commands.elevator;

import frc.robot.subsystems.Elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorManualPIDCommand extends CommandBase{
    private Elevator elevator;
    private DoubleSupplier speed;

	public ElevatorManualPIDCommand(Elevator elevator, DoubleSupplier speed) {
        this.elevator = elevator;
        this.speed = speed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        elevator.manual_PID(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
