/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class RotateToLLTargetTimed extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RotateToLLTargetTimed(double timeout) {
    requires(DrivetrainSubsystem.getInstance());
    addSequential(new RotateToLLTarget(), timeout);
  }
}
