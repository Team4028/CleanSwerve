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
import com.swervedrivespecialties.exampleswerve.commands.drive.VictorySpin;
import com.swervedrivespecialties.exampleswerve.commands.drive.VictorySpins;
import com.swervedrivespecialties.exampleswerve.commands.shooter.AutoInfeed;
import com.swervedrivespecialties.exampleswerve.commands.shooter.RunFeeder;
import com.swervedrivespecialties.exampleswerve.commands.shooter.ShootBalls;
import com.swervedrivespecialties.exampleswerve.commands.shooter.ShootFrisbee;
import com.swervedrivespecialties.exampleswerve.commands.shooter.ShootFrisbees;
import com.swervedrivespecialties.exampleswerve.commands.shooter.StartShooterAtSpeed;
import com.swervedrivespecialties.exampleswerve.commands.shooter.StopShooter;
import com.swervedrivespecialties.exampleswerve.commands.shooter.TogglePunches;
import com.swervedrivespecialties.exampleswerve.commands.shooter.ToggleSucc;
import com.swervedrivespecialties.exampleswerve.util.InertiaGain;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class testAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public testAuto() {
    setInterruptible(false);
    addSequential(new FollowTrajectory(Trajectories.testAutoTrajectoryOneSupplier));
    addSequential(new AutoInfeed(false));
    addSequential(new FollowTrajectory(Trajectories.testAutoTrajectoryTwoSupplier), 3);
    addSequential(new StartShooterAtSpeed(.6));
    addSequential(new RotateToLLTarget(), 1);
    addSequential(new WaitCommand(.5));
    addSequential(new ShootBalls(4));
    addSequential(new WaitCommand(.5));
    addSequential(new StopShooter());
    addParallel(new ToggleSucc());
    addParallel(new AutoInfeed(true));
    addSequential(new FollowTrajectory(Trajectories.testAutoTrajectoryThreeSupplier, new InertiaGain(0, 0, .015)));
    addSequential(new ToggleSucc());
    addSequential(new FollowTrajectory(Trajectories.testAutoTrajectoryFourSupplier, new InertiaGain(0, 0, .01)));
    addParallel(new StartShooterAtSpeed(.6));
    addSequential(new RotateToLLTarget(), 1);
    addSequential(new TogglePunches());
    addSequential(new WaitCommand(.3));
    addSequential(new TogglePunches());
    addSequential(new WaitCommand(.7));
    addSequential(new TogglePunches());
    addSequential(new WaitCommand(.3));
    addSequential(new TogglePunches());
    addSequential(new WaitCommand(.7));
    addSequential(new TogglePunches());
    addSequential(new WaitCommand(.3));
    addSequential(new TogglePunches());
    addSequential(new StopShooter());
    addSequential(new FollowTrajectory(Trajectories.testAutoTrajectoryFiveSupplier));
    addSequential(new AutoInfeed(false));
    addSequential(new FollowTrajectory(Trajectories.testAutoTrajectorySixSupplier));
    addSequential(new VictorySpin());
    //addSequential(new LineDrive(new Vector2(0, -48), true, Rotation2.fromDegrees(270)));
    // addSequential(new VictorySpins(5));
  }
}

