/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands;

import java.util.function.Supplier;

import com.swervedrivespecialties.exampleswerve.autonomous.Trajectories;

import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class LineDrive extends CommandGroup {

  /**
   * Add your docs here.
   */
  public LineDrive(Vector2 vec, Rotation2 rot) {
    Vector2 driveVector = vec;
    Trajectory lineTraj = Trajectories.generateLineTrajectory(driveVector, rot);
    Supplier<Trajectory> lineTrajSupplier = () -> lineTraj;
    addSequential(new FollowTrajectory(lineTrajSupplier));
  }
}
