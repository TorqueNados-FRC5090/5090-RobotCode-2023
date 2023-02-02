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
  final Drivetrain m_robotDrive = new Drivetrain();

  public final FieldSim m_fieldSim = new FieldSim(m_robotDrive);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  // The driver's controller

  static XboxController driver = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

    m_fieldSim.initSim();
    initializeAutoChooser();
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        m_robotDrive.setDefaultCommand(
        new SetSwerveDrive(
          m_robotDrive,
          () -> driver.getLeftY(),
          () -> driver.getLeftX(),
          () -> driver.getRightX()));

        JoystickButton button_8 = new JoystickButton(driver, 8);
        button_8.whenPressed(new ToggleFieldOriented(m_robotDrive));
  }

  private void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
    m_autoChooser.addOption("Drive Forward", new DriveForward(m_robotDrive));
    m_autoChooser.addOption("5 Ball Auto", new FiveBallAuto(m_robotDrive));

    SmartDashboard.putData("Auto Selector", m_autoChooser);
  }

  public void simulationPeriodic() {
    m_fieldSim.periodic();
    periodic();
  }

  public void periodic() {
    m_fieldSim.periodic();
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }

}
