package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command autonCommand;

  private RobotContainer robotContainer;

  // This function is run when the robot is first started up and should be used
  // for any initialization code.
  @Override
  public void robotInit() {
    if (RobotBase.isReal())
      DataLogManager.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  // This function is called once at the start of auton
  @Override
  public void autonomousInit() {
    autonCommand = robotContainer.getAutonomousCommand();

    if (autonCommand != null) {
      autonCommand.schedule();
    }
  }

  // This function is called every 20ms during auton
  @Override
  public void autonomousPeriodic() {
  }

  // This function is called once at the start of teleop
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous command stops when teleop starts
    if (autonCommand != null) {
      autonCommand.cancel();
    }

    // new SetSwerveOdometry(m_robotContainer.m_robotDrive,
    // m_robotContainer.m_fieldSim,new Pose2d(6.13, 5.23,
    // Rotation2d.fromDegrees(-41.5))).schedule();
  }

  // This function is called every 20ms during teleop
  @Override
  public void teleopPeriodic() {
  }

  // This function is called every 20ms no matter what
  @Override
  public void robotPeriodic() {
    // Runs the Command Scheduler.
    CommandScheduler.getInstance().run();
    robotContainer.m_fieldSim.periodic();
    robotContainer.periodic();    
  }

  @Override
  public void simulationPeriodic() {
    robotContainer.m_fieldSim.periodic();
    robotContainer.simulationPeriodic();
  }

}
