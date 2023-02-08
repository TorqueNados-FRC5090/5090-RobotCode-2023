package frc.robot.commands.auto;

// Pathplanner imports
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// Command imports
import frc.robot.commands.swerve.SetSwerveIdleMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Other imports
import frc.robot.subsystems.Drivetrain;
import frc.robot.lists.Constants.DriveConstants;
import edu.wpi.first.math.util.Units;

// The purpose of this command is to drive the robot two feet forwards.
public class DriveForward extends SequentialCommandGroup {

  public DriveForward(Drivetrain swerveDrive) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("DriveForward", Units.feetToMeters(2),
            Units.feetToMeters(2), false);
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
            trajectory,
            swerveDrive::getPoseMeters,
            DriveConstants.kSwerveKinematics,
            swerveDrive.getXPidController(),
            swerveDrive.getYPidController(),
            swerveDrive.getThetaPidController(),
            swerveDrive::setSwerveModuleStatesAuto,
            swerveDrive);
        addCommands(
            new SetSwerveOdometry(swerveDrive, trajectory.getInitialPose()),
            command,
            new SetSwerveIdleMode(swerveDrive, false)
                .andThen(() -> swerveDrive.drive(0, 0, 0, false)));
  }
}
