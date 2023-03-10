package frc.robot.commands;

// Imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.DriveConstants.ROTATION_SLEW;
import static frc.robot.Constants.DriveConstants.ROTATION_DEADBAND;

/** Rotates the robot to 180 degrees and resets heading */
public class Head180 extends CommandBase {
    // Declare variables that will be initialized by the constructor
    private final Drivetrain drivetrain;
    private final SlewRateLimiter slewRot = new SlewRateLimiter(ROTATION_SLEW);

    /** 
     * Constructs a Head180 command
     *  
     * @param drivetrain The robot's drivetrain
    */
    public Head180(Drivetrain drivetrain) {
        // Initialize internal variables with values passed through params
        this.drivetrain = drivetrain;
        
        // Tell the CommandBase that this command uses the drivetrain
        addRequirements(drivetrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Ask the drivetrain to calculate a rotational input
        double rot = drivetrain.head(180);

        // If the rotation input is between -.1 and .1, set it to 0
        double deadbandedRot = MathUtil.applyDeadband(Math.abs(rot),
                ROTATION_DEADBAND) * Math.signum(rot);

        // Square value after deadband while keeping original sign
        deadbandedRot = -Math.signum(deadbandedRot) * Math.pow(deadbandedRot, 2);

        // Apply a slew rate to the input, limiting the robot's rotational acceleration
        double slewedRot = slewRot.calculate(deadbandedRot);

        // Send the processed output to the drivetrain        
        drivetrain.drive(0, 0, slewedRot, true);
    }

    // Command ends when robot is done rotating
    @Override
    public boolean isFinished() {
        return drivetrain.headingPIDAtTarget();
    }

    // Reset the heading of the robot at the end of the command
    @Override
    public void end(boolean interrupted) { drivetrain.resetHeading(); }
}
