package com.swervedrivespecialties.exampleswerve.commands.drive;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.drivers.Gyroscope;
import edu.wpi.first.wpilibj.command.Command;

public class ZeroGyro extends Command 
{
  Gyroscope _gyro = DrivetrainSubsystem.getInstance().getGyroscope();
  
  public ZeroGyro() 
  {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
    _gyro.setAdjustmentAngle(_gyro.getUnadjustedAngle());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() 
  {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() 
  {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() 
  {
  }
}