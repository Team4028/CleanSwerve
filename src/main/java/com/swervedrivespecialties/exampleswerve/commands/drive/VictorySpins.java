/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class VictorySpins extends CommandGroup {
  /**
   * Add your docs here.
   */
  public VictorySpins(int num) {
    for (int ind = 0; ind < num; ind++){
      addSequential(new VictorySpin());
    }
  }
}
