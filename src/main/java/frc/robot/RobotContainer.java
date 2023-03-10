package frc.robot;

import static frc.robot.Constants.ControllerPorts.DRIVER_PORT;
import static frc.robot.Constants.DIOPorts.CLAW_LASER_PORT;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutonContainer;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LockDrivetrain;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

/** This is where the drivetrain will be controlled */
public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private AutonContainer auton = new AutonContainer(drivetrain);
    private Claw claw = new Claw(CLAW_LASER_PORT);
    private XboxController driverController = new XboxController(DRIVER_PORT);
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
            new DriveCommand(drivetrain, 
            () -> driverController.getLeftX(), 
            () -> driverController.getLeftY(),
            () -> drivetrain.head(0)));
        
        Trigger lockHeading180Btn = new Trigger(() -> driverController.getLeftTriggerAxis() > .2);
        lockHeading180Btn.whileTrue(
            new DriveCommand(drivetrain, 
            () -> driverController.getLeftX(), 
            () -> driverController.getLeftY(),
            () -> drivetrain.head(180)));
    }

    /** Initialize the auton selector on the dashboard */
    private void initChooser() {
        autonChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
        autonChooser.addOption("Cone Cube with No Bump", auton.coneCubeNoBumpAuto());
        autonChooser.addOption("Cube Cube with No Bump", auton.cubeCubeNoBumpAuto());
        autonChooser.addOption("Cone Cube with Bump", auton.coneCubeBumpAuto());

        SmartDashboard.putData("Auton Selector", autonChooser);
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Auton for going over the line
        return autonChooser.getSelected();
    }

    /** @return The robot's drivetrain */
    public Drivetrain getDrivetrain() { return drivetrain; }
    public Claw getClaw() { return claw; }

    // For running TimedRobot style code in RobotContainer
    /** Should always be called from Robot.teleopPeriodic() */
    public void teleopPeriodic() {
        if(driverController.getStartButtonPressed())
            drivetrain.toggleFieldCentric();
            
        if(driverController.getBackButtonPressed())
            drivetrain.resetHeading();

        if(driverController.getRightBumperPressed())
            claw.toggleClaw();
    }
}
