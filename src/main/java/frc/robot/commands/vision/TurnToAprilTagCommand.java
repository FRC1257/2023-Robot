package frc.robot.commands.vision;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

import frc.robot.Constants;
import static frc.robot.Constants.Drivetrain.*;

public class TurnToAprilTagCommand extends CommandBase {
  private Drivetrain drivetrain;
  private Vision vision;

  double targetAngle; // Future variable, probably used in the case where the camera is not placed horizionally in front of the robot
  double kp; // scaling ratio for robot movement
  double error; // amount of error our robot detects that it tries to correct for, relative to the position of the AprilTag

  /** Turns the robot using the gyro only for following an April Tag (tracked target from VisionSubsystem) */
  public TurnToAprilTagCommand(Drivetrain drivetrain, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain; // define drive and vision system
    this.vision = vision;
    kp = TRACKED_TAG_ROTATION_KP; // Calculate kp scaling ratio based off scaled GYRO_KP value (used in execute())
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.getHasTarget()) { // if we find a target,
      error = vision.getBestTarget().getYaw(); // calculate error based off Yaw value of our current best target
      double value = -Math.min(error*kp, 1); // calculate motor percentage value

      drivetrain.manualDrive(-value, value); // write values to motors, negative and positive value in order for turning to occur
    } else {
      drivetrain.manualDrive(0, 0); // otherwise, don't do anything
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("ENDED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return Math.abs(error) < 1;
  } 
}
