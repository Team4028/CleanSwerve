/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj.command.Command;

public class RunFeeder extends Command {
  public RunFeeder() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  Shooter _shooter = Shooter.getInstance();
  int kMaxCycles = 3;
  int numCycles;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    numCycles = 0;
    _shooter.setShouldRunFeeder(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    numCycles++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return numCycles > kMaxCycles;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    _shooter.setShouldRunFeeder(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
