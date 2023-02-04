package frc.robot.commands.swerve;

// Imports
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.simulation.FieldSim;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Drivetrain;

// The purpose of this command is to tell the robot where it is
public class SetSwerveOdometry extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private final Drivetrain swerveDrive;
    private final FieldSim fieldSim;
    private final Pose2d pose2d;

    /**
     * Sets the robot's position
     *
     * @param drivetrain A drivetrain object to be driven.
     * @param pose The robot's position.
     */
    public SetSwerveOdometry(Drivetrain swerveDrive, Pose2d pose) {
        this(swerveDrive, null, pose); // Call other constructor
    }

    /**
     * Sets the robot's position
     *
     * @param drivetrain A drivetrain object to be driven.
     * @param fieldSim FieldSim object to be used during simulation.
     * @param pose The robot's position.
     */
    public SetSwerveOdometry(Drivetrain swerveDrive, FieldSim fieldSim, Pose2d pose) {
        if (RobotBase.isSimulation() && fieldSim == null)
            System.out.println(
                "SetOdometry Command Error: Robot is in Simulation, but you did not add FieldSim to the argument");

        // Initialize internal variables with parameter values
        this.swerveDrive = swerveDrive;
        this.fieldSim = fieldSim;
        this.pose2d = pose;
        
        // Declare a swerve drivetrain dependency
        addRequirements(swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerveDrive.setOdometry(pose2d);
        // driveTrain.setNavXOffset(pose2d.getRotation().getDegrees());
        // if (RobotBase.isSimulation()) fieldSim.resetRobotPose(pose2d);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() { return true; }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
