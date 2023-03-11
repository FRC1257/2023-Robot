package frc.robot.commands.elevator;

import frc.robot.subsystems.Elevator.Elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorManualCommand extends CommandBase{
    private Elevator elevator;
    private DoubleSupplier speedSupplier;
	public ElevatorManualCommand(Elevator elevator, DoubleSupplier speedSupplier) {
        this.elevator = elevator;
        this.speedSupplier = speedSupplier;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        elevator.manualControl(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}