/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;


import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RotateToAngle extends Command {

  private static DrivetrainSubsystem _drivetrainSubsystem = DrivetrainSubsystem.getInstance();
  private PidController _pidController = new PidController(new PidConstants(0.007, 0, .0005));//#TODO tune to be more aggressive.
  private double _currentTime, _target;
  private double kAcceptableError = 0.8;

  public RotateToAngle(double angle) {
    requires(_drivetrainSubsystem);
    _target = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _currentTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double minSpeedScale = DrivetrainSubsystem.getInstance().getMinSpeed();

    //All of the holonomic drive command, fwd, strafe, and rot are scaled by speedScale.
    double controllerSpeedScale = Robot.getOi().getRawSpeedScaleCmd();
    //square the speed scale trigger, and minimize it at .4
    controllerSpeedScale = minSpeedScale + (1 - minSpeedScale) * controllerSpeedScale * controllerSpeedScale;

      double forward = -Robot.getOi().getRawForwadCmd();
    // Square the forward stick
    forward = Math.copySign(Math.pow(forward, 2.0), forward) * controllerSpeedScale;

    double strafe = -Robot.getOi().getRawStrafeCmd();
    // Square the strafe stick
    strafe = Math.copySign(Math.pow(strafe, 2.0), strafe) * controllerSpeedScale;
    double localTime = Timer.getFPGATimestamp();
    double deltaTime = localTime - _currentTime;
    _currentTime = localTime;
    double err = getMinAngleDiff(_drivetrainSubsystem.getGyroscope().getAngle().toDegrees(), _target);
    _drivetrainSubsystem.holonomicDrive(new Vector2(forward,strafe), -Math.copySign(.05, err) +  _pidController.calculate(err, deltaTime), true); //.055
    SmartDashboard.putNumber("AngleError", _target - _drivetrainSubsystem.getGyroscope().getAngle().toDegrees());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Math.abs( _target - _drivetrainSubsystem.getGyroscope().getAngle().toDegrees()) < kAcceptableError){
      return true;
    }
    else{
      return false;
    }
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

  public double getMinAngleDiff(double a1, double a2){
    double rawDiff = Math.toRadians(a1 - a2);
    double c = Math.cos(rawDiff);
    double s = Math.sin(rawDiff);
    double minDiff = Math.atan2(s, c);
    return Math.toDegrees(minDiff);
  }
}
