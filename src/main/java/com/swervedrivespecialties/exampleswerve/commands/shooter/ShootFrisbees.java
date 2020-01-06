/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ShootFrisbees extends CommandGroup {
  double kBetweenShotWaitTime = .7;

  public ShootFrisbees(int numShots) {
    for (int ind = 0; ind < numShots - 1; ind++){
      addSequential(new ShootFrisbee());
      addSequential(new WaitCommand(kBetweenShotWaitTime));
    }
    addSequential(new ShootFrisbee());
  }
}