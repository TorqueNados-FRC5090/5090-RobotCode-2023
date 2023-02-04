package frc.robot.commands.swerve;

// Imports
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/** Sets the drivetrain to neutral (coast/brake) */
public class SetSwerveIdleMode extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final Drivetrain drivertain;
    private final boolean brake;

    /**
     * Sets the drivetrain neutral mode (coast/brake).
     *
     * @param drivetrain The driveTrain used by this command.
     * @param brake Whether brakes should be active in neutral mode.
     */
    public SetSwerveIdleMode(Drivetrain drivetrain, boolean brake) {
        // Initialize internal variables with parameter values
        this.drivertain = drivetrain;
        this.brake = brake;

        // Declare a swerve drivetrain dependency
        addRequirements(drivertain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivertain.setIdleMode(brake);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true; // End immediately
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
