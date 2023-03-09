package frc.robot;

// Import Constants
import static frc.robot.Constants.ArmIDs.*;
import frc.robot.Constants.ArmConstants.ArmState;
import static frc.robot.Constants.ControllerPorts.OPERATOR_PORT;

// Camera imports
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

// Subsystem and subclass imports
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.misc_subclasses.Dashboard;
import frc.robot.misc_subclasses.Limelight;

// Command imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Misc imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;


public class Robot extends TimedRobot {
    private RobotContainer robotContainer;

    // Commands
    private Command autonCommand;

    // Subsystem and subclass objects
    private Arm arm;
    private Claw claw;
    private Dashboard dashboard;
    private Limelight limelight;
    
    // Other objects
    private XboxController operatorController;
    private Compressor compressor;
    private double autonStartTime;

    // This function is run when the robot is first started up and should be used
    // for any initialization code.
    @Override
    public void robotInit() {
        // Start the camera feed
        CameraServer.startAutomaticCapture();

        // Construct objects
        robotContainer = new RobotContainer();
        operatorController = new XboxController(OPERATOR_PORT);
        arm = new Arm(ROTATION_ID, ROTATION_FOLLOWER_ID, TELESCOPE_ID, TELESCOPE_FOLLOWER_ID, SLIDER_ID);
        claw = robotContainer.getClaw();
        limelight = new Limelight();
        dashboard = new Dashboard();
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    }

    // This function is called once at the start of auton
    @Override
    public void autonomousInit() {
        // Get the command to be used in auton
        autonCommand = robotContainer.getAutonomousCommand();
        autonStartTime = Timer.getFPGATimestamp();
        // Schedule the command if there is one
        if (autonCommand != null)
            autonCommand.schedule();
    }

    // This function is called every 20ms during auton
    @Override
    public void autonomousPeriodic() { 
        double currentTime = Timer.getFPGATimestamp() - autonStartTime;

        // Start by lifting the arm to dropoff position
        if (currentTime < .2)
            arm.goTo(ArmState.DROPOFF_MED);

        // Drop the preloaded piece
        if (currentTime > 2 && currentTime < 2.2)
            claw.open();

        // Return the arm to zero
        if (currentTime > 3 && currentTime < 3.2)
            arm.goTo(ArmState.ZERO);
    }
    
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

        /*     ___________________________
         *    |  __0__                    |
         *    | |  |  |  (1) [3] [4] (5)  |
         *    | |_/_\_|  (2) [6] [x] (x)  |
         *    |___________________________|
         * 
         *    1 - Pickup human
         *    2 - Pickup floor
         * 
         *    3 - Dropoff low / Intermediate
         *    4 - Dropoff medium
         *    5 - Dropoff high
         *    
         *    6 - Zero
         */

        // Pressing Right bumper makes the arm go to the preset of the drop of high position
        if (operatorController.getRightBumper())
            arm.goTo(ArmState.DROPOFF_HIGH);

        // Pressing Y makes the arm go to the preset of the drop of medium position
        if (operatorController.getYButtonPressed())
            arm.goTo(ArmState.DROPOFF_MED);

        // Pressing Left trigger makes the arm go to the preset of the floor pickup position
        if (operatorController.getXButtonPressed())
            arm.goTo(ArmState.INTERMEDIATE);

        // Pressing A makes the arm go to the preset of the zero position
        if (operatorController.getAButtonPressed())
            arm.goTo(ArmState.ZERO);

        // Pressing Left bumper makes the arm go to the preset of the human pickup position
        if (operatorController.getLeftBumperPressed())
            arm.goTo(ArmState.PICKUP_HUMAN);
        
        // Pressing Left trigger makes the arm go to the preset of the floor pickup position
        if (operatorController.getLeftTriggerAxis() > 0)
            arm.goTo(ArmState.PICKUP_FLOOR);

    }

    // This function is called every 20ms while the robot is enabled
    @Override
    public void robotPeriodic() {    
        // Print data to the dashboard
        dashboard.printLimelightData(limelight);
        dashboard.printBasicDrivetrainData(robotContainer.getDrivetrain());
        dashboard.printArmData(arm);

        // Run any functions that always need to be running
        limelight.updateLimelightTracking();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void testPeriodic() {

        /*     ___________________________
         *    |  __0__                    |
         *    | |  |  |  (1) [3] [4] (7)  |
         *    | |_/_\_|  (2) [5] [6] (8)  |
         *    |___________________________|
         * 
         *    1 and 2 control claw
         *    3 and 4 control telescope
         *    5 and 6 control slider
         *    7 and 8 control rotation
         */
        
        if (operatorController.getLeftBumper())
            claw.open();
        else if (operatorController.getRightTriggerAxis() > .5)
            claw.close();
            
        if(operatorController.getXButton())
            arm.getTelescopeMotor().set(.1);
        else if(operatorController.getYButton())
            arm.getTelescopeMotor().set(-.1);
        else
            arm.getTelescopeMotor().set(0); 

        if(operatorController.getAButton())
            arm.getSliderMotor().set(.15);
        else if(operatorController.getBButton())
            arm.getSliderMotor().set(-.15);
        else
            arm.getSliderMotor().set(0); 

        if(operatorController.getRightBumper())
            arm.getRotationMotor().set(.05);
        else if(operatorController.getRightTriggerAxis() > .5)
            arm.getRotationMotor().set(-.05);
        else
            arm.getRotationMotor().set(0); 
    }
}