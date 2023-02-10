package frc.robot;

// Controller Imports
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;

// Command imports
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Camera imports
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.misc_subclasses.Limelight;

// Subsystem imports
import frc.robot.subsystems.Drivetrain;

// Misc imports
import frc.robot.misc_subclasses.Dashboard;
import edu.wpi.first.wpilibj.DataLogManager;

public class Robot extends TimedRobot {
    // Commands
    private Command autonCommand;
    private RobotContainer robotContainer;

    // Controller ojects
    private Joystick joystick; 
    private XboxController xbox;

    // Subsystem objects
    private Drivetrain drivetrain;
    private Dashboard dashboard;
    private Limelight limelight;

    // This function is run when the robot is first started up and should be used
    // for any initialization code.
    @Override
    public void robotInit() {
        if (RobotBase.isReal())
            DataLogManager.start();

        // Initialize variables
        joystick = new Joystick(0);
        xbox  = new XboxController(1);

        CameraServer.startAutomaticCapture();
        limelight = new Limelight();

        dashboard = new Dashboard();

        robotContainer = new RobotContainer();
    }

    // This function is called once at the start of auton
    @Override
    public void autonomousInit() {
        autonCommand = robotContainer.getAutonomousCommand();

        if (autonCommand != null)
            autonCommand.schedule();
    }

    // This function is called every 20ms during auton
    @Override
    public void autonomousPeriodic() { 
        
    }
    
    // This function is called once at the start of teleop
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous command stops when teleop starts
        if (autonCommand != null)
            autonCommand.cancel();
    }

    // This function is called every 20ms during teleop
    @Override
    public void teleopPeriodic() {
    
    }

    // This function is called every 20ms while the robot is enabled
    @Override
    public void robotPeriodic() {    
        // Update subclass internal values
        limelight.updateLimelightTracking();

        // Update dashboard
        dashboard.printLimelightData(limelight);

        // Runs the Command Scheduler.
        CommandScheduler.getInstance().run();
        robotContainer.fieldSim.periodic();
        robotContainer.periodic();  
    }

    @Override
    public void simulationPeriodic() {
        robotContainer.fieldSim.periodic();
        robotContainer.simulationPeriodic();
    }
}