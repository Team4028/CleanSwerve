/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands;

import java.util.Arrays;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class QuickMath extends Command {
  DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();
  double kSpeed = 1;
  double[] dubArr = new double[100];
  int count = 0;

  public QuickMath() {
    requires(_drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    _drive.holonomicDrive(new Vector2(kSpeed, 0), 0, false);
    dubArr[count] = _drive.getKinematicVelocity().length;
    count++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return count == 100;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Arrays.sort(dubArr);
    double var = .2 * (dubArr[99] + dubArr[98] + dubArr[97] + dubArr[96] + dubArr[95]);
    SmartDashboard.putNumber("Double Array Top", var);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
