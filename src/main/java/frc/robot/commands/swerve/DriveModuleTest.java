package frc.robot.commands.swerve;

// Imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lists.Constants.DriveConstants.ModulePosition;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

// The purpose of this command is to test the driving motors in a swerve chassis
// by executing only the drive portion of a swerve command. (Does not control turning motors)
public class DriveModuleTest extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drivetrain drivetrain;
    private boolean individual;
    private final DoubleSupplier throttleInput, strafeInput, rotationInput, testInput;

    /**
     * @param drivetrain A drivetrain object to be tested.
     * @param throttleInput Movement instruction along the horizontal axis.
     * @param strafeInput Movement instruction along the vertical axis.
     * @param rotationInput Rotational instruction. Positive is clockwise.
     * @param testInput Purely hypothetical instruction for testing.
     * @param individual When true, each motor uses one input. When false, all motors use the throttle input.
     */
    public DriveModuleTest(
        Drivetrain drivetrain,
        DoubleSupplier throttleInput, 
        DoubleSupplier strafeInput, 
        DoubleSupplier rotationInput, 
        DoubleSupplier testInput, 
        boolean individual) {

        // Initialize internal variables with parameter values
        this.drivetrain = drivetrain;
        this.throttleInput = throttleInput;
        this.strafeInput = strafeInput;
        this.rotationInput = rotationInput;
        this.testInput = testInput;
        this.individual = individual;

        // Declare a dependency on a swerve drivetrain
        addRequirements(drivetrain);
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

        // When individual is false, all motors use the throttle input. 
        // When individual is true, each motor uses one input.
        if (!individual) {
            drivetrain.driveModule(ModulePosition.FRONT_LEFT, throttle);
            drivetrain.driveModule(ModulePosition.FRONT_RIGHT, throttle);
            drivetrain.driveModule(ModulePosition.BACK_LEFT, throttle);
            drivetrain.driveModule(ModulePosition.BACK_RIGHT, throttle);
        } 
        else {
            drivetrain.driveModule(ModulePosition.FRONT_LEFT, throttle);
            drivetrain.driveModule(ModulePosition.FRONT_RIGHT, strafe);
            drivetrain.driveModule(ModulePosition.BACK_LEFT, rotation);
            drivetrain.driveModule(ModulePosition.BACK_RIGHT, test);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Command never ends
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // If the command is interrupted, stop driving.
        drivetrain.driveModule(ModulePosition.FRONT_LEFT,0);
        drivetrain.driveModule(ModulePosition.FRONT_RIGHT, 0);
        drivetrain.driveModule(ModulePosition.BACK_RIGHT, 0);
        drivetrain.driveModule(ModulePosition.BACK_LEFT, 0);
    }
}
