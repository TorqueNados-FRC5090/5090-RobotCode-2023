package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/** Drives the robot */
public class LockDrivetrain extends CommandBase {
    // Declare variables that will be initialized by the constructor
    private final Drivetrain drivetrain;

    /** 
     * Constructs a LockDrivetrain command
     *  
     * @param drivetrain The robot's drivetrain
    */
    public LockDrivetrain(Drivetrain drivetrain) {
        // Initialize internal variables with values passed through params
        this.drivetrain = drivetrain;
        
        // Tell the CommandBase that this command uses the drivetrain
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.lock();
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
