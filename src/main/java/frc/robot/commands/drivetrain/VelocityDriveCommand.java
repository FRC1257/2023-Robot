package frc.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;


import static frc.robot.Constants.Drivetrain.*;

public class VelocityDriveCommand extends CommandBase {
    
    private final Drivetrain drivetrain;
    private final DoubleSupplier speedForwardSupplier;
    private final DoubleSupplier speedTurnSupplier;
    private final SlewRateLimiter limiter;
    private final BooleanSupplier visionSupplier;
    private boolean useVision;

    public VelocityDriveCommand(Drivetrain drivetrain, DoubleSupplier speedForwardSupplier,
        DoubleSupplier speedTurnSupplier, BooleanSupplier visionSupplier, boolean useVision) {

        this.drivetrain = drivetrain;
        this.speedForwardSupplier = speedForwardSupplier;
        this.speedTurnSupplier = speedTurnSupplier;
        this.limiter = new SlewRateLimiter(DRIVE_CLOSED_MAX_ACC);
        this.visionSupplier = visionSupplier;
        this.useVision = useVision;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double visionAdd = 0;

        // limit acceleration of the linear speed (create separate one for turning speed if needed)
        drivetrain.velocityDrive(limiter.calculate(speedForwardSupplier.getAsDouble()), speedTurnSupplier.getAsDouble() + visionAdd);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
