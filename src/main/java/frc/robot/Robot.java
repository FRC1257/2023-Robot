package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    
    private RobotContainer robotContainer;
    private Command autoCommand;
    
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        PortForwarder.add(5800, "photonvision.local", 5800);

        if (isSimulation()) {
            System.out.println("Running in simulation");
        } else {
            System.out.println("Running on robot");
        }
    
        /* var instance = NetworkTableInstance.getDefault();
        PhotonCamera camera = new PhotonCamera(instance, "Microsoft-LifeCam-3000");
         */
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.displayShuffleboard();
        if(SmartDashboard.getBoolean("Testing", false)) {
            robotContainer.tuningPeriodic();
        }
    }

    @Override
    public void autonomousInit() {
        autoCommand = robotContainer.getAutoCommand();

        if(autoCommand != null) {
            autoCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        robotContainer.stopDisplayingTraj();
        if(autoCommand != null) {
            autoCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        robotContainer.tuningInit();
    }

    @Override
    public void disabledPeriodic() {
        // stop all motors
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}