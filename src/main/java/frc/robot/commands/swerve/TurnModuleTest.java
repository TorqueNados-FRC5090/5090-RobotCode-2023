package frc.robot.commands.swerve;

// Imports
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;
import frc.robot.lists.Constants.DriveConstants.ModulePosition;
import frc.robot.subsystems.Drivetrain;

// The purpose of this command is to test the turning motors in a swerve chassis
// by executing a swerve command on turning motors. (Does not control driving motors)
public class TurnModuleTest extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Drivetrain swerveDrive;
    private final DoubleSupplier throttleInput, strafeInput, rotationInput, testInput;

    /**
     * @param swerveDriveSubsystem The subsystem used by this command.
     * @param throttleInput Movement instruction along the horizontal axis.
     * @param strafeInput Movement instruction along the vertical axis.
     * @param rotationInput Rotational instruction. Positive is clockwise.
     * @param testInput Purely hypothetical instruction for testing.
     */
    public TurnModuleTest(
        Drivetrain swerveDriveSubsystem,
        DoubleSupplier throttleInput,
        DoubleSupplier strafeInput,
        DoubleSupplier rotationInput, 
        DoubleSupplier testInput) {

        // Initialize internal variables with parameter values
        swerveDrive = swerveDriveSubsystem;
        this.throttleInput = throttleInput;
        this.strafeInput = strafeInput;
        this.rotationInput = rotationInput;
        this.testInput = testInput;

        // Declare a dependency on a swerve drivetrain
        addRequirements(swerveDriveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // If any input is between -.05 and .05, set it to 0
        double throttle = MathUtil.applyDeadband(Math.abs(throttleInput.getAsDouble()), 0.05)
            * Math.signum(throttleInput.getAsDouble());
        double strafe = MathUtil.applyDeadband(Math.abs(strafeInput.getAsDouble()), 0.05)
            * Math.signum(strafeInput.getAsDouble());
        double rotation = MathUtil.applyDeadband(Math.abs(rotationInput.getAsDouble()), 0.05)
            * Math.signum(rotationInput.getAsDouble());
        double test = MathUtil.applyDeadband(Math.abs(testInput.getAsDouble()), 0.05)
            * Math.signum(testInput.getAsDouble());

        // Reduce each instruction by 50%
        throttle *= .5;
        strafe *= .5;
        rotation *= .5;
        test *= .5;

        // Each motor is individually controlled by one of the instructions
        swerveDrive.turnModule(ModulePosition.FRONT_LEFT, throttle);
        swerveDrive.turnModule(ModulePosition.FRONT_RIGHT, strafe);
        swerveDrive.turnModule(ModulePosition.REAR_LEFT, rotation);
        swerveDrive.turnModule(ModulePosition.REAR_RIGHT, test);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Command never ends
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // If the command is interrupted, stop turning.
        swerveDrive.turnModule(ModulePosition.FRONT_LEFT, 0);
        swerveDrive.turnModule(ModulePosition.FRONT_RIGHT, 0);
        swerveDrive.turnModule(ModulePosition.REAR_RIGHT, 0);
        swerveDrive.turnModule(ModulePosition.REAR_LEFT, 0);
    }
}
