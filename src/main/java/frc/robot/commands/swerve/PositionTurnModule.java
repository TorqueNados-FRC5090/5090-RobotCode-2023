package frc.robot.commands.swerve;

// Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.subsystems.Drivetrain;

// The purpose of this command is to turn a swerve module to face some position
public class PositionTurnModule extends CommandBase {

    private Drivetrain drivetrain;
    private ModulePosition mp;

    /**
     * @param drivetrain A drivetrain object whose modules will be turned.
     * @param mp The position of the module. (See ModulePosition enum)
     */
    public PositionTurnModule(Drivetrain drivetrain,  ModulePosition mp) {
        // Initialize internal variables with parameter values
        this.drivetrain = drivetrain;
        this.mp = mp;
    
        // Declare a dependency on a swerve drivetrain
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 
        drivetrain.targetAngle = drivetrain.getAnglefromThrottle();
        SmartDashboard.putNumber("TargetAngle", drivetrain.targetAngle);
        drivetrain.positionTurnModule(mp, drivetrain.targetAngle);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // End command when module has reached its target
        return drivetrain.getTurnInPosition(mp, drivetrain.targetAngle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // When the command ends, stop turning.
        drivetrain.turnModule(ModulePosition.FRONT_LEFT, 0);
        drivetrain.turnModule(ModulePosition.FRONT_RIGHT, 0);
        drivetrain.turnModule(ModulePosition.BACK_RIGHT, 0);
        drivetrain.turnModule(ModulePosition.BACK_LEFT, 0);
    }
}
