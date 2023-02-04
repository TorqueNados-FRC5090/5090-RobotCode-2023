package frc.robot.commands;

// Imports
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

// The purpose of this command is to toggle the drive mode
// between robot-centric and field-centric
public class ToggleFieldOriented extends InstantCommand {
    private Drivetrain drive;

    /**
     * @param drivetrain A drivetrain object to be toggled.
     */
    public ToggleFieldOriented(Drivetrain drive) {
        this.drive = drive;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drive.fieldOriented = !drive.fieldOriented;
    }
}
