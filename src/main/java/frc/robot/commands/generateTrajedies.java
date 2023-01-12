package frc.robot.commands;
import ToChargeCommand;
import ToCargoCommand;
import ToScoreCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.Drivetrain.*;

public class generateTrajedies extends SequentialCommandGroup {
 private boolean charge; 
 private boolean score;
 private boolean cargo; 
 private Drivetrain driveTrain; 
 private double maxVelocity;
 private double minVelocity;
 private final Pose2d StartPos;
 private final Pose2d NewPos;
 
  public generateTrajedies (boolean charge, boolean score, boolean cargo, Drivetrain driveTrain, double maxVelocity, double minVelocity, Pose2d StartPos, Pose2d NewPos) {
   this.charge = charge;
   this.score = score;
   this.cargo = cargo;
   this.driveTrain = driveTrain
   this.maxVelocity = maxVelocity;
   this.minVelocity = minVelocity;
   this.StartPos = StartPos;
   this.NewPos = NewPos;
   
  }
 public void TrajediesDecider() {
   if (this.score){
    ToScoreCommands obj1 = new ToScoreCommands(this.driveTrain, this.StartPos, this.NewPos);  
    
     } 
   if (this.cargo){
    ToCargoCommands obj1 = new ToCargoCommands(this.driveTrain, this.StartPos, this.NewPos);
    
      }
   if (this.charge){
    ToChargeCommands obj1 = new ToChargeCommands(this.driveTrain, this.StartPos);
    
      }
   if (this.StartPos != this.NewPos){
    ToPos obj1 = new ToPos(this.StartPos, this.NewPos);
     }
   
     } 
  
   }
 
}
