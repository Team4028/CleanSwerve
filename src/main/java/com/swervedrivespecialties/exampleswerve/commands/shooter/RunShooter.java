/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj.command.Command;

public class RunShooter extends Command {

  Shooter _shooter = Shooter.getInstance();

  public RunShooter() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(_shooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _shooter.runShooter(_shooter.getShouldRunShooter());
    _shooter.runFeeder(_shooter.getShouldRunFeeder());
    _shooter.runInfeed(Robot.getOi().getInfeedCmd());
    _shooter.resetPuncher();
    _shooter.resetSucc();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    _shooter.runShooter(_shooter.getShouldRunShooter());
    _shooter.runFeeder(_shooter.getShouldRunFeeder());
    _shooter.runInfeed(Robot.getOi().getInfeedCmd());
  }


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
