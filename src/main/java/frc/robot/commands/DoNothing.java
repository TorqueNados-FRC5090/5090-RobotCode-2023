// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/** A version of the WPILib WaitCommand that actively forces the drivetrain to do nothing */
public class DoNothing extends CommandBase {
    private Timer timer = new Timer();
    private final double duration;
    private final Drivetrain drivetrain;

    /**
     * Creates a new DoNothing. This command will do nothing, and end after the specified duration.
     *
     * @param seconds the time to wait, in seconds
     */
    public DoNothing(double seconds, Drivetrain drivetrain) {
        duration = seconds;
        this.drivetrain = drivetrain;  
        
        // Tell the CommandBase that this command uses the drivetrain
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override // Force the drivetrain do nothing
    public void execute() { drivetrain.drive(0, 0, 0, true); }

    @Override // Stop the timer
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override // Command ends when timer has reached the given time
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }
}
