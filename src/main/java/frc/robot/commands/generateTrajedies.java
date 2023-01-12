package frc.robot.commands;
import ToChargeCommand;
import ToCargoCommand;
import ToScoreCommand;

public class generateTrajedies {
 private boolean charge; 
 private boolean score
 private boolean cargo; 
 private double maxVelocity;
 private double minVelocity;
 private final Pose2d StartPos;
 private final Pose2d NewPos;
 
  public (boolean charge, boolean score, boolean cargo, Pose2d StartPos, Pose2d NewPos) {
   this.charge = charge;
   this.score = score;
   this.cargo = cargo
   this.StartPos = StartPos;
   this.NewPos = NewPos; 
   
  }
 public void TrajediesDecider() {
   if (this.score = true){
      
    
   } 
   if (this.cargo = true){
    ToCargoCommands obj1 = new ToCargoCommands(this.StartPos, this.NewPos, this.minVelocity, this.maxVelocity);
    
   }
   if (this.charge = true){
    ToChargeCommands obj1 = new ToChargeCommands(this.StartPos, this.minVelocity, this.maxVelocity);
    
   }
   
   
   } 
  
  }
 
}
