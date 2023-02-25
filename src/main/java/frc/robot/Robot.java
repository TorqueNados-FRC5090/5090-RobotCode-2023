package frc.robot;

// Command imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Camera imports
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.misc_subclasses.Limelight;

// Subsystem and subclass imports
import frc.robot.subsystems.Arm;

// Import Constants
import static frc.robot.Constants.ArmIDs.*;
import static frc.robot.Constants.ControllerPorts.OPERATOR_PORT;

// Misc imports
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.misc_subclasses.Dashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;

    // Commands
    private Command autonCommand;

    // Subsystem and subclass objects
    private Arm arm;
    private Dashboard dashboard;
    private Limelight limelight;
    
    // Other objects
    private XboxController operatorController;
    private Compressor compressor;

    // Used to test the arm
    private double rotationPos;
    private double sliderPos;
    private double telescopePos;
    private int prev;

    // This function is run when the robot is first started up and should be used
    // for any initialization code.
    @Override
    public void robotInit() {
        // Start the camera feed
        CameraServer.startAutomaticCapture();

        // Construct objects
        robotContainer = new RobotContainer();
        operatorController = new XboxController(OPERATOR_PORT);
        arm = new Arm(ROTATION_ID, TELESCOPE_ID, TELESCOPE_FOLLOWER_ID, SLIDER_ID);
        limelight = new Limelight();
        dashboard = new Dashboard();
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    }

    // This function is called once at the start of auton
    @Override
    public void autonomousInit() {
        // Get the command to be used in auton
        autonCommand = robotContainer.getAutonomousCommand();
        // Schedule the command if there is one
        if (autonCommand != null)
            autonCommand.schedule();
    }

    // This function is called every 20ms during auton
    @Override
    public void autonomousPeriodic() {}
    
    // This function is called once at the start of teleop
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous command stops when teleop starts
        if (autonCommand != null)
            autonCommand.cancel();

        // Start the compressor
        compressor.enableDigital();
    }

    // This function is called every 20ms during teleop
    @Override
    public void teleopPeriodic() {
        robotContainer.teleopPeriodic();
    }

    // This function is called once at the start of a test
    @Override
    public void testInit() {
        // Init values for arm testing
        rotationPos = 0;
        telescopePos = 0;
        sliderPos = 0;
        prev = -1;
    }

    // This function is called every 20ms while testing
    @Override
    public void testPeriodic() {
        // Get the state of the dpad
        int curr = operatorController.getPOV();
    
        // Arm's telescope is controlled by the bumpers
        if (operatorController.getLeftBumperPressed())
            arm.telescopeGoTo(--telescopePos);
        else if (operatorController.getRightBumperPressed())
            arm.telescopeGoTo(++telescopePos);

        // Arm's slider is controlled by left and right dpad
        if (curr == 270 && curr != prev)
            arm.sliderGoTo(--sliderPos);
        else if (curr == 90 && curr != prev)
            arm.sliderGoTo(++sliderPos);
        
        // Arm's rotation is controlled by up and down dpad
        if (curr == 180 && curr != prev)
            arm.rotationGoTo(--rotationPos);
        else if (curr == 0 && curr != prev)
            arm.rotationGoTo(++rotationPos);

        // Pressing B resets the arm to where it was when the robot was powered on
        if (operatorController.getBButtonPressed())
            arm.zeroPosition();

        // The dpad's previous value is updated
        prev = curr; 
    }

    // This function is called every 20ms while the robot is enabled
    @Override
    public void robotPeriodic() {    
        // Print data to the dashboard
        dashboard.PIDtoDashboard(arm.getRotationPid(), "rotation");
        dashboard.PIDtoDashboard(arm.getTelescopePid(), "telescope");
        dashboard.PIDtoDashboard(arm.getSliderPid(), "slider");
        dashboard.printLimelightData(limelight);
        dashboard.printBasicDrivetrainData(robotContainer.getDrivetrain());

        // Run any functions that always need to be running
        CommandScheduler.getInstance().run();
        limelight.updateLimelightTracking();
    }
}