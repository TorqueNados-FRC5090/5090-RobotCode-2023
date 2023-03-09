package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.SwerveConstants.SWERVE_KINEMATICS;

public class AutonCommandsContainer {
    private Drivetrain drivetrain;

    public AutonCommandsContainer(Drivetrain drivetrain) {this.drivetrain = drivetrain;}

    public Command coneCubeNoBumpAuto() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("ConeCubeNoBumpAuto", 4.5, 4.5, false);

        //PPSwerveControllerCommand pathFollowerCommand = new PPSwerveControllerCommand(
        return new PPSwerveControllerCommand(
            trajectory, 
            drivetrain::getPoseMeters, 
            SWERVE_KINEMATICS, 
            drivetrain.getXPidController(), 
            drivetrain.getYPidController(), 
            drivetrain.getThetaPidController(),
            drivetrain::setModuleStates,
            drivetrain);
    }
}
