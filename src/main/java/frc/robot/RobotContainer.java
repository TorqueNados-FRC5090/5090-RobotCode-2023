package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.ControllerPorts.DRIVER_PORT;

/** This is where the drivetrain will be controlled */
public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private XboxController driver = new XboxController(DRIVER_PORT);

    /** Constructs a RobotContainer */
    public RobotContainer() {
        // If the drivetrain is not busy, drive using joysticks
        drivetrain.setDefaultCommand(
            new DriveCommand(drivetrain, 
            () -> driver.getLeftX(), 
            () -> driver.getLeftY(),
            () -> driver.getRightX())
        );
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

    // For running TimedRobot style code in RobotContainer
    /** Should always be called from Robot.teleopPeriodic() */
    public void teleopPeriodic() {
        if(driver.getStartButtonPressed())
            drivetrain.toggleFieldCentric();
            
        if(driver.getBackButtonPressed())
            drivetrain.resetHeading();
    }
}
