package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.SwerveConstants.SWERVE_KINEMATICS;

public class AutonContainer {
    private Drivetrain drivetrain;

    /** Used in auton to automatically adjust for inaccuracies in the robot's movement along the X axis */
    private PIDController xController =
        new PIDController(3, 0, .05);
    /** Used in auton to automatically adjust for inaccuracies in the robot's movement along the Y axis */
    private PIDController yController =
        new PIDController(2.6, 0, .0005);
    /** Used in auton to automatically adjust for inaccuracies in the robot's rotation */
    private PIDController thetaController =
        new PIDController(0, 0, 0);

    public AutonContainer(Drivetrain drivetrain) {this.drivetrain = drivetrain;}


    /** Auton for no bump side that scores a preloaded cone and a floor cube */
    public Command coneCubeNoBumpAuto() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("ConeCubeNoBumpAuto", 4.5, 4.5, false);
        PPSwerveControllerCommand pathFollowerCommand = new PPSwerveControllerCommand(
            trajectory, 
            drivetrain::getPoseMeters, 
            SWERVE_KINEMATICS, 
            xController,
            yController,
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);

        return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetHeading()),
            new InstantCommand(() -> drivetrain.setOdometry(trajectory.getInitialHolonomicPose())),
            pathFollowerCommand);
    }

    /** Auton for no bump side that scores a preloaded cube and a floor cube */
    public Command cubeCubeNoBumpAuto() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("CubeCubeNoBumpAuto", 4.5, 4.5, false);
        PPSwerveControllerCommand pathFollowerCommand = new PPSwerveControllerCommand(
            trajectory, 
            drivetrain::getPoseMeters, 
            SWERVE_KINEMATICS, 
            xController,
            yController,
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);

        return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetHeading()),
            new InstantCommand(() -> drivetrain.setOdometry(trajectory.getInitialHolonomicPose())),
            pathFollowerCommand);
    }

    /** Auton for bump side that scores a preloaded cone and a floor cube */
    public Command coneCubeBumpAuto() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("ConeCubeBumpAuto", 4.5, 4.5, false);
        PPSwerveControllerCommand pathFollowerCommand = new PPSwerveControllerCommand(
            trajectory, 
            drivetrain::getPoseMeters, 
            SWERVE_KINEMATICS, 
            xController,
            yController,
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);

        return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetHeading()),
            new InstantCommand(() -> drivetrain.setOdometry(trajectory.getInitialHolonomicPose())),
            pathFollowerCommand);
    }

    /** 3 piece auton bump side:
     *  preload cone
     *  pickup and score cube high
     *  pickup and score another cube mid */
    public Command noBumpSide3PieceAuton() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("ConeCubeNoBumpAuto", 3, 3, false);
        PPSwerveControllerCommand part1Drive = new PPSwerveControllerCommand(
            trajectory, 
            drivetrain::getPoseMeters, 
            SWERVE_KINEMATICS, 
            xController,
            yController,
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);

        PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("NoBumpExtraCubeExtension", 3, 3, false);
        PPSwerveControllerCommand part2Drive = new PPSwerveControllerCommand(
            trajectory2, 
            drivetrain::getPoseMeters, 
            SWERVE_KINEMATICS, 
            xController,
            yController,
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);

        return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetHeading()),
            new InstantCommand(() -> drivetrain.setOdometry(trajectory.getInitialHolonomicPose())),
            part1Drive,
            new DoNothing(5, drivetrain),
            part2Drive
            );
    }

    /** Auton for bump side that scores a preloaded cone and a floor cube */
    public Command testAuto(String autoName, double maxSpeed, double maxAccel) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(autoName, maxSpeed, maxAccel, false);

        PPSwerveControllerCommand pathFollowerCommand = new PPSwerveControllerCommand(
            trajectory, 
            drivetrain::getPoseMeters, 
            SWERVE_KINEMATICS, 
            xController,
            yController,
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);

        return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetHeading()),
            new InstantCommand(() -> drivetrain.setOdometry(trajectory.getInitialHolonomicPose())),
            pathFollowerCommand);
    }
}
