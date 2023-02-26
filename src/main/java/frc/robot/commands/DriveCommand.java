package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.DriveConstants.*;

/** Drives the robot */
public class DriveCommand extends CommandBase {
    // Declare variables that will be initialized by the constructor
    private final Drivetrain drivetrain;
    private final DoubleSupplier inputY;
    private final DoubleSupplier inputX;
    private final DoubleSupplier inputRot;

    // Declare and initialize the limiters used to slew instructions
    private final SlewRateLimiter slewX = new SlewRateLimiter(TRANSLATION_SLEW);
    private final SlewRateLimiter slewY = new SlewRateLimiter(TRANSLATION_SLEW);
    private final SlewRateLimiter slewRot = new SlewRateLimiter(ROTATION_SLEW);

    /** 
     * Constructs a DriveCommand
     *  
     * @param drivetrain The robot's drivetrain
     * @param translationInputX The left/right translation instruction
     * @param translationInputY The forward/back translation instruction
     * @param rotation The rotational instruction
    */
    public DriveCommand(Drivetrain drivetrain, DoubleSupplier translationInputX, DoubleSupplier translationInputY, DoubleSupplier rotationInput) {

        // Initialize internal variables with values passed through params
        this.drivetrain = drivetrain;
        this.inputX = translationInputX;
        this.inputY = translationInputY;
        this.inputRot = rotationInput;
        
        // Tell the CommandBase that this command uses the drivetrain
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // If a translation input is between -.05 and .05, set it to 0
        double deadbandedX = MathUtil.applyDeadband(Math.abs(inputX.getAsDouble()),
            TRANSLATION_DEADBAND) * Math.signum(inputX.getAsDouble());
        double deadbandedY = MathUtil.applyDeadband(Math.abs(inputY.getAsDouble()),
            TRANSLATION_DEADBAND) * Math.signum(inputY.getAsDouble());

        // If the rotation input is between -.1 and .1, set it to 0
        double deadbandedRot = MathUtil.applyDeadband(Math.abs(inputRot.getAsDouble()),
            ROTATION_DEADBAND) * Math.signum(inputRot.getAsDouble());

        // Square values after deadband while keeping original sign
        deadbandedX = -Math.signum(deadbandedX) * Math.pow(deadbandedX, 2);
        deadbandedY = -Math.signum(deadbandedY) * Math.pow(deadbandedY, 2);
        deadbandedRot = -Math.signum(deadbandedRot) * Math.pow(deadbandedRot, 2);

        // Apply a slew rate to the inputs, limiting the rate at which the robot changes speed
        double slewedX = slewX.calculate(deadbandedX);
        double slewedY = slewY.calculate(deadbandedY);
        double slewedRot = slewRot.calculate(deadbandedRot);

        // Send the processed output to the drivetrain
        drivetrain.drive(slewedX, slewedY, slewedRot, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Has no end condition
    }
}
