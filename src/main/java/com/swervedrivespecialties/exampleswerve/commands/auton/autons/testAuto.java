/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.auton.autons;

import com.swervedrivespecialties.exampleswerve.autonomous.Trajectories;
import com.swervedrivespecialties.exampleswerve.commands.auton.FollowTrajectory;
import com.swervedrivespecialties.exampleswerve.commands.auton.LineDrive;
import com.swervedrivespecialties.exampleswerve.commands.drive.RotateToLLTarget;
import com.swervedrivespecialties.exampleswerve.commands.drive.VictorySpins;
import com.swervedrivespecialties.exampleswerve.commands.shooter.AutoInfeed;
import com.swervedrivespecialties.exampleswerve.commands.shooter.ShootBalls;
import com.swervedrivespecialties.exampleswerve.commands.shooter.ShootFrisbees;
import com.swervedrivespecialties.exampleswerve.commands.shooter.ToggleSucc;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class testAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public testAuto() {
    addSequential(new FollowTrajectory(Trajectories.testAutoTrajectoryOneSupplier));
    addSequential(new AutoInfeed(false));
    addSequential(new FollowTrajectory(Trajectories.testAutoTrajectoryTwoSupplier));
    addSequential(new RotateToLLTarget());
    addSequential(new ShootBalls(4));
    addParallel(new ToggleSucc());
    addParallel(new AutoInfeed(true));
    addSequential(new FollowTrajectory(Trajectories.testAutoTrajectoryThreeSupplier));
    addSequential(new ToggleSucc());
    addSequential(new FollowTrajectory(Trajectories.testAutoTrajectoryFourSupplier));
    addSequential(new RotateToLLTarget());
    addSequential(new ShootFrisbees(4));
    addSequential(new FollowTrajectory(Trajectories.testAutoTrajectoryFiveSupplier));
    addSequential(new AutoInfeed(false));
    addSequential(new LineDrive(new Vector2(0, -48), true, Rotation2.fromDegrees(270)));
    addSequential(new VictorySpins(5));
  }
}

