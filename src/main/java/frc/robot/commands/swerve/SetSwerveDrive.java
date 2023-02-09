package frc.robot.commands.swerve;

// Imports
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import java.util.function.DoubleSupplier;
import frc.robot.lists.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

// The purpose of this command is to drive a swerve chassis
public class SetSwerveDrive extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drivetrain drivetrain;
    private final SlewRateLimiter slewX = new SlewRateLimiter(DriveConstants.TRANSLATION_SLEW);
    private final SlewRateLimiter slewY = new SlewRateLimiter(DriveConstants.TRANSLATION_SLEW);
    private final SlewRateLimiter slewRot = new SlewRateLimiter(DriveConstants.ROTATION_SLEW);
    private final DoubleSupplier throttleInput, strafeInput, rotationInput;

    /**
     * @param drivetrain A drivetrain object to be driven.
     * @param throttleInput Movement instruction along the horizontal axis.
     * @param strafeInput Movement instruction along the vertical axis.
     * @param rotationInput Rotational instruction. Positive is clockwise.
     */
    public SetSwerveDrive(
        Drivetrain drivetrain,
        DoubleSupplier throttleInput,
        DoubleSupplier strafeInput,
        DoubleSupplier rotationInput) {

        // Initialize internal variables with parameter values
        this.drivetrain = drivetrain;
        this.throttleInput = throttleInput;
        this.strafeInput = strafeInput;
        this.rotationInput = rotationInput;
        
        // Declare a swerve drivetrain dependency
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // If throttle input is between -.05 and .05, set it to 0
        double throttle = MathUtil.applyDeadband(Math.abs(throttleInput.getAsDouble()),
            DriveConstants.TRANSLATION_DEADBAND) * Math.signum(throttleInput.getAsDouble());

        // If strafe input is between -.05 and .05, set it to 0
        double strafe = MathUtil.applyDeadband(Math.abs(strafeInput.getAsDouble()),
            DriveConstants.TRANSLATION_DEADBAND) * Math.signum(strafeInput.getAsDouble());

        // If rotation input is between -.1 and .1, set it to 0
        double rotation = MathUtil.applyDeadband(Math.abs(rotationInput.getAsDouble()),
            DriveConstants.ROTATION_DEADBAND) * Math.signum(rotationInput.getAsDouble());

        // Square values after deadband while keeping original sign
        throttle = -Math.signum(throttle) * Math.pow(throttle, 2);
        strafe = -Math.signum(strafe) * Math.pow(strafe, 2);
        rotation = -Math.signum(rotation) * Math.pow(rotation, 2);

        // Applies a slew rate to the inputs, limiting the rate at which the robot changes speed
        double throttle_sl = slewX.calculate(throttle);
        double strafe_sl = slewY.calculate(strafe);
        double rotation_sl = slewRot.calculate(rotation);

        // Send the processed output command
        drivetrain.drive(throttle_sl, strafe_sl, rotation_sl, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Command never ends
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
