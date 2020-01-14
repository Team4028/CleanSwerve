/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.util.util;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.Command;

public class mikeeDrive extends Command {
  DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();

  double kDeadBand = .05;
  double kRotate = .1;

  public mikeeDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(_drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {        
    double rotation = -Robot.getOi().getRawRotationCmd();
    rotation = util.signum(rotation) * (util.epsilonEquals(rotation, kDeadBand) ? 0 : kRotate);
    _drive.holonomicDrive(Vector2.ZERO, rotation);
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
