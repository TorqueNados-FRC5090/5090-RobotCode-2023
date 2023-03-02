package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForward extends CommandBase{
    
   private Drivetrain drivetrain;

   private double distance; 
   private double target;


   /**constructor
    * @param drivetrain creates dependency on drive train
    * @param target creates a target that can be changed 
    */
    public DriveForward(Drivetrain drivetrain, double target){
        
        this.drivetrain = drivetrain;
        this.target = target;
        
        addRequirements(drivetrain);

    }

@Override
    public void initialize(){
        drivetrain.resetOdometry();
    }    

@Override
    public void execute(){
        drivetrain.drive(0, .4 * Math.signum(target), 0, true);
        distance = drivetrain.getPoseMeters().getTranslation().getX();
    }

@Override
    public boolean isFinished(){
        return Math.abs(distance) >= target;
    }

@Override
    public void end(boolean interrupted){
        drivetrain.resetOdometry();
    }
}
