package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LockDrivetrain;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Claw;

import static frc.robot.Constants.ControllerPorts.DRIVER_PORT;
import static frc.robot.Constants.DIOPorts.CLAW_LASER_PORT;

/** This is where the drivetrain will be controlled */
public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Claw claw = new Claw(CLAW_LASER_PORT);
    private XboxController driverController = new XboxController(DRIVER_PORT);

    /** Constructs a RobotContainer */
    public RobotContainer() {
        // If the drivetrain is not busy, drive using joysticks
        drivetrain.setDefaultCommand(
            new DriveCommand(drivetrain, 
            () -> driverController.getLeftX(), 
            () -> driverController.getLeftY(),
            () -> driverController.getRightX())
        );

        Trigger lockBtn = new Trigger(() -> driverController.getXButton());
        lockBtn.whileTrue(new LockDrivetrain(drivetrain));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new DriveCommand(drivetrain, () -> 0, () -> .5, () -> 0);
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

        if(driverController.getLeftBumperPressed())
            claw.open();

        else if (driverController.getRightBumperPressed())
            claw.close();
    }
}
