/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class AutoInfeed extends Command {

  boolean isInfeed;
  double kAutoInfeedTimeout = 1.2;
  double kAutoOutfeedTimeout = .3;
  double startTime;
  double kTimeout;

  Shooter shooter = Shooter.getInstance();

  public AutoInfeed(boolean shouldInfeed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    isInfeed = shouldInfeed;
    kTimeout = shouldInfeed ? kAutoInfeedTimeout : kAutoOutfeedTimeout;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
    shooter.startAuto(isInfeed);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > kTimeout;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    shooter.endAuto();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
