package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ToggleFieldOriented;
import frc.robot.commands.auto.DriveForward;
import frc.robot.commands.auto.FiveBallAuto;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    // The robot's subsystems
    final Drivetrain robotDrive = new Drivetrain();

    public final FieldSim fieldSim = new FieldSim(robotDrive);

    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    // The driver's controller

    static XboxController driver = new XboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

        fieldSim.initSim();
        initializeAutoChooser();
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            robotDrive.setDefaultCommand(
            new SetSwerveDrive(
            robotDrive,
            () -> driver.getLeftY(),
            () -> driver.getLeftX(),
            () -> driver.getRightX()));

            JoystickButton button_8 = new JoystickButton(driver, 8);
            button_8.whenPressed(new ToggleFieldOriented(robotDrive));
    }

    private void initializeAutoChooser() {
        autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
        autoChooser.addOption("Drive Forward", new DriveForward(robotDrive));
        autoChooser.addOption("5 Ball Auto", new FiveBallAuto(robotDrive));

        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    public void simulationPeriodic() {
        fieldSim.periodic();
        periodic();
    }

    public void periodic() {
        fieldSim.periodic();
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }

}
