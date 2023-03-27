package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;

public class AutoCommandDecorator extends CommandBase {
  private CommandBase command;
  public AutoCommandDecorator(CommandBase command) {
     this.command = command;
  }
  
  @Override
  public void initialize() { command.initialize(); }
  
  @Override
  public void execute() { command.execute(); }
  
  @Override 
  public void end(boolean interrupted) { command.end(interrupted); }
  
  @Override
  public boolean isFinished() {
    return DriverStation.isTeleop() || command.isFinished();
  }
}
