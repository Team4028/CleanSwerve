/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;


import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RotateToLLTarget extends Command {

  DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();
  private double _theta, _rotateCommand;

  private double kAcceptableError = 0.8;
  private double ff = .11;
  private double p = .01;

  

  private NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");
	private NetworkTableEntry tx = nt.getEntry("tx");

  public RotateToLLTarget() {
    requires(_drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    _theta = tx.getDouble(0);
    _rotateCommand = Math.abs(_theta) > kAcceptableError ? -Math.copySign(Math.pow(Math.copySign(ff, _theta) + (p * _theta), 2), _theta): 0;


    //THis doesn't work well because of the nature of the rotation loop here, still in development. 

    // double minSpeedScale = DrivetrainSubsystem.getInstance().getMinSpeed();

    // //All of the holonomic drive command, fwd, strafe, and rot are scaled by speedScale.
    // double controllerSpeedScale = Robot.getOi().getRawSpeedScaleCmd();
    // //square the speed scale trigger, and minimize it at .4
    // controllerSpeedScale = minSpeedScale + (1 - minSpeedScale) * controllerSpeedScale * controllerSpeedScale;

    // double forward = -Robot.getOi().getRawForwadCmd();
    // // Square the forward stick
    // forward = Math.copySign(Math.pow(forward, 2.0), forward) * controllerSpeedScale;

    // double strafe = -Robot.getOi().getRawStrafeCmd();
    // // Square the strafe stick
    // strafe = Math.copySign(Math.pow(strafe, 2.0), strafe) * controllerSpeedScale;

    _drive.holonomicDrive(new Vector2(0, 0), _rotateCommand);

    SmartDashboard.putNumber("AngleError", _theta - _drive.getGyroscope().getAngle().toDegrees());
    SmartDashboard.putNumber("LL Value", _theta);
    SmartDashboard.putNumber("Rotate Command", _rotateCommand);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(_theta - _drive.getGyroscope().getAngle().toDegrees()) < kAcceptableError;
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
