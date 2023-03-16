package frc.robot;

// Import constants
import static frc.robot.Constants.ControllerPorts.DRIVER_PORT;
import static frc.robot.Constants.DIOPorts.CLAW_LASER_PORT;
import static frc.robot.Constants.ArmIDs.*;

// Command imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutonContainer;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveWithHeading;
import frc.robot.commands.LockDrivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

// Other imports
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Handles everything command based */
public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Arm arm = new Arm(ROTATION_ID, ROTATION_FOLLOWER_ID, TELESCOPE_ID, TELESCOPE_FOLLOWER_ID, SLIDER_ID);
    private final Claw claw = new Claw(CLAW_LASER_PORT);
    private final AutonContainer auton = new AutonContainer(drivetrain, arm, claw);
    private final XboxController driverController = new XboxController(DRIVER_PORT);
    private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

    /** Constructs a RobotContainer */
    public RobotContainer() {
        initChooser();

        // If the drivetrain is not busy, drive using joysticks
        drivetrain.setDefaultCommand(
            new DriveCommand(drivetrain, 
            () -> driverController.getLeftX(), 
            () -> driverController.getLeftY(),
            () -> driverController.getRightX())
        );

        Trigger lockBtn = new Trigger(() -> driverController.getXButton());
        lockBtn.whileTrue(new LockDrivetrain(drivetrain));

        Trigger lockHeadingZeroBtn = new Trigger(() -> driverController.getRightTriggerAxis() > .2);
        lockHeadingZeroBtn.whileTrue(
            new DriveWithHeading(drivetrain, 
            () -> driverController.getLeftX(), 
            () -> driverController.getLeftY(),
            0));
        
        Trigger lockHeading180Btn = new Trigger(() -> driverController.getLeftTriggerAxis() > .2);
        lockHeading180Btn.whileTrue(
            new DriveWithHeading(drivetrain, 
            () -> driverController.getLeftX(), 
            () -> driverController.getLeftY(),
            180));

        Trigger slowDriveBtn = new Trigger(() -> driverController.getLeftStickButton());
        slowDriveBtn.whileTrue(
            new DriveCommand(drivetrain, 
            () -> driverController.getLeftX() * .4, 
            () -> driverController.getLeftY() * .4,
            () -> driverController.getRightX() * .4));
    }

    /** Initialize the auton selector on the dashboard */
    private void initChooser() {
        autonChooser.setDefaultOption("Do Nothing", new WaitCommand(0));

        SmartDashboard.putData("Auton Selector", autonChooser);
        autonChooser.setDefaultOption("Drop High Auto", auton.dropHigh());
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // For testing
        //return auton.testAuto(testAutonChooser.getSelected(), 1, 1);

        return autonChooser.getSelected();
    }

    /** @return The robot's drivetrain */
    public Drivetrain getDrivetrain() { return drivetrain; }
    public Claw getClaw() { return claw; }
    public Arm getArm() {return arm;}

    // For running TimedRobot style code in RobotContainer
    /** Should always be called from Robot.teleopPeriodic() */
    public void teleopPeriodic() {
        if(driverController.getStartButtonPressed())
            drivetrain.toggleFieldCentric();
            
        if(driverController.getBackButtonPressed())
            drivetrain.resetHeading();

        if(driverController.getRightBumperPressed())
            claw.toggleClaw();
        else if(!driverController.getRightBumper())
            claw.autoGrab();
    }
}
