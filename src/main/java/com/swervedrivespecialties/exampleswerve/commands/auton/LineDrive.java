/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.auton;

import java.awt.Robot;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class LineDrive extends CommandGroup {

  static final double kDefaultTimeout = 4;

  /**
   * Add your docs here.
   */
  public LineDrive(Vector2 vec, boolean isFieldOriented, Rotation2 rot, double timeOut){
    setInterruptible(false);
    if (isFieldOriented){
      addSequential(new FieldOrientedLineDrive(vec, rot), timeOut);
    }else{
      addSequential(new RobotOrientedLineDrive(vec, rot), timeOut);
    }
  }

  public LineDrive(Vector2 vec, boolean isFieldOriented, Rotation2 rot){
    this(vec, isFieldOriented, rot, kDefaultTimeout);
  }

  public LineDrive(Vector2 vec, boolean isFieldOriented, double timeOut){
    this(vec, isFieldOriented, null, timeOut);
  }

  public LineDrive(Vector2 vec, boolean isFieldOriented){
    this(vec, isFieldOriented, null, kDefaultTimeout);
  }

  public LineDrive(Vector2 vec){
    this(vec, false, null, kDefaultTimeout);
  }
}
