package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class generateTrajedies extends SequentialCommandGroup {
    private boolean charge;
    private boolean score;
    private boolean cargo;
    private Drivetrain driveTrain;
    private final Pose2d StartPos;
    private final Pose2d NewPos;

    public generateTrajedies(boolean charge, boolean score, boolean cargo, Drivetrain driveTrain, Pose2d StartPos,
            Pose2d NewPos) {
        this.charge = charge;
        this.score = score;
        this.cargo = cargo;
        this.driveTrain = driveTrain;
        this.StartPos = StartPos;
        this.NewPos = NewPos;

    }

    public void TrajediesDecider() {
        if (this.score) {
            ToScoreCommand obj1 = new ToScoreCommand(this.driveTrain, this.StartPos, this.NewPos);

        }
        if (this.cargo) {
            ToCargoCommand obj1 = new ToCargoCommand(this.driveTrain, this.StartPos, this.NewPos);

        }
        if (this.charge) {
            ToChargeCommand obj1 = new ToChargeCommand(this.driveTrain, this.StartPos);

        }
        if (this.StartPos != this.NewPos) {
            ToPos obj1 = new ToPos(this.driveTrain, this.StartPos, this.NewPos);
        }

    }

}
