/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands;

import java.util.Optional;
import java.util.function.Supplier;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.util.InertiaGain;

import org.frcteam2910.common.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FollowTrajectory extends Command {
  DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();

  double kFinishRotatingTimeout = 1.5;

  ///////////// PATH FOLLOWING CONSTANTS //////////////////
  private static final double kMaxVelo = 12 * 12; //This is the physical max velocity of the machine, not of any path
  private static final double kInterceptVoltage = .018809; //the physical minimum voltage to make the robot move forward
  private static final double kPathFollowingAccelFeedForward = 0;
  private static final double kPathFollowingTranslationP = .06;
  private static final double kPathFollowingTranslationI = 0;
  private static final double kPathFollowingTranslationD = 0;
  private static final double kPathFollowingRotationP = .45;
  private static final double kPathFollowingRotationI = 0;
  private static final double kPathFollowingRotationD = .00025;
  ////////////////////////////////////////////////////////////

  //create appropriate constant classes and eventually a TrajectoryFollower from constants given above
  PidConstants translationConstants = new PidConstants(kPathFollowingTranslationP, kPathFollowingTranslationI, kPathFollowingTranslationD);
  PidConstants rotationConstants = new PidConstants(kPathFollowingRotationP, kPathFollowingRotationI, kPathFollowingRotationD);
  double kFeedForwardVelocity = (1 - kInterceptVoltage) / kMaxVelo;
  DrivetrainFeedforwardConstants feedforwardConstants = new DrivetrainFeedforwardConstants(kFeedForwardVelocity, kPathFollowingAccelFeedForward, kInterceptVoltage);
  HolonomicFeedforward feedforward = new HolonomicFeedforward(feedforwardConstants); //can have separate forward and strafe feed forwards if desired. 
  HolonomicMotionProfiledTrajectoryFollower follower;

  InertiaGain inertiaGain;
  Supplier<Trajectory> trajectorySupplier;
  Trajectory trajectory;

  double timestamp;
  double lastTimestamp;
  double dt;

  public FollowTrajectory(Supplier<Trajectory> trajSupplier, InertiaGain iGain) {
    requires(_drive);
    trajectorySupplier = trajSupplier;
    inertiaGain = iGain;
  }

  public FollowTrajectory(Supplier<Trajectory> trajSupplier){
    this(trajSupplier, InertiaGain.id);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    follower = new HolonomicMotionProfiledTrajectoryFollower(translationConstants, rotationConstants, feedforward);
    trajectory = trajectorySupplier.get();
    timestamp = Timer.getFPGATimestamp();
    resetDriveKinematics();
    follower.follow(trajectory);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    updateTime();
    Optional<HolonomicDriveSignal> driveSignal = calculate();
    if (driveSignal.isPresent()){
      holonomicDrive(inertiaGain.apply(driveSignal.get()));
      double fwdCmd = driveSignal.get().getTranslation().x;
      double stfCmd = driveSignal.get().getTranslation().y;
      double rotCmd = driveSignal.get().getRotation();
      SmartDashboard.putNumber("fwd Cmd Path Following", fwdCmd);
      SmartDashboard.putNumber("stf Cmd Path Following", stfCmd);
      SmartDashboard.putNumber("rot Cmd Path Following", rotCmd);
    } else {
      _drive.holonomicDrive(Vector2.ZERO, 0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return follower.getCurrentTrajectory().isEmpty();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Command finishRotateCommand = new RotateToAngleTimed(trajectory.calculateSegment(trajectory.getDuration()).rotation.toDegrees(), kFinishRotatingTimeout);
    finishRotateCommand.start();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
    follower.cancel();
  }


  ////////////// Drivetrain-based utilities ///////////////
  private RigidTransform2 getPose(){
    return new RigidTransform2(_drive.getKinematicPosition(), _drive.getGyroscope().getAngle());
  }

  private void updateTime(){
    lastTimestamp = timestamp;
    timestamp = Timer.getFPGATimestamp();
    dt = timestamp - lastTimestamp;
  }

  private Optional<HolonomicDriveSignal> calculate(){
    return follower.update(getPose(), _drive.getKinematicVelocity(), _drive.getGyroscope().getRate(), timestamp, dt);
  }

  private void holonomicDrive(HolonomicDriveSignal sig){
    _drive.holonomicDrive(sig.getTranslation(), sig.getRotation(), sig.isFieldOriented());
  }

  private void resetDriveKinematics(){
    _drive.resetKinematics(Vector2.ZERO, Timer.getFPGATimestamp());
  }
}
