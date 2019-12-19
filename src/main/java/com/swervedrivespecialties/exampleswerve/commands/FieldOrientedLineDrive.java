/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands;

import java.util.function.Supplier;

import com.swervedrivespecialties.exampleswerve.autonomous.Trajectories;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.Command;

public class FieldOrientedLineDrive extends Command {
  
  DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();
  Vector2 driveVector;
  Rotation2 driveRotation;

  public FieldOrientedLineDrive(Vector2 vec, Rotation2 rot) {
    driveVector = vec;
    driveRotation = rot;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(_drive);
  }

  public FieldOrientedLineDrive(Vector2 vec){
      this(vec, null);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Supplier<Trajectory>lineTrajSupplier;
    Rotation2 curRot = _drive.getGyroscope().getAngle(); 
    if (driveRotation == null){
      Trajectory lineTraj = Trajectories.generateLineTrajectory(driveVector, curRot, curRot);
      lineTrajSupplier = () -> lineTraj;
    } else {
      Trajectory lineTraj = Trajectories.generateLineTrajectory(driveVector, curRot, driveRotation);
      lineTrajSupplier = () -> lineTraj;
    }
    Command driveCommand = new FollowTrajectory(lineTrajSupplier);
    driveCommand.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
