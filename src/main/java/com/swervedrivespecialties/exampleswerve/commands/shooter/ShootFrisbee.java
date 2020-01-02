/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ShootFrisbee extends CommandGroup {
  /**
   * Add your docs here.
   */

   double kFrisbeeShootTime = .3;

  public ShootFrisbee() {
    addSequential(new TogglePunches());
    addSequential(new WaitCommand(kFrisbeeShootTime));
    addSequential(new TogglePunches());
  }
}
