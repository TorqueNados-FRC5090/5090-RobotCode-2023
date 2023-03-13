package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/** Locks the wheels of the drivetrain into an X shape for anti-defense */
public class LockDrivetrain extends CommandBase {
    // Variable declaration
    private final Drivetrain drivetrain;
    private final SwerveModuleState[] lockedStates = {
        new SwerveModuleState(0, new Rotation2d(-45)),
        new SwerveModuleState(0, new Rotation2d(45)),
        new SwerveModuleState(0, new Rotation2d(-45)),
        new SwerveModuleState(0, new Rotation2d(45))
    };

    /** 
     * Constructs a LockDrivetrain command
     *  
     * @param drivetrain The robot's drivetrain
    */
    public LockDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        // Tell the CommandBase that this command uses the drivetrain
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    
    @Override // Lock the drivetrain
    public void execute() {
        drivetrain.setModuleStates(lockedStates);
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Command never ends on its own
    }
}
