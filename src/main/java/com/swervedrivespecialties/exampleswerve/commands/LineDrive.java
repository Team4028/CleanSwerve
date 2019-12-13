/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.util.util;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class LineDrive extends Command {

  DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();
  Vector2 _vector;
  public static Vector2 _targetVec;
  boolean _isFieldOriented;
  double _time;
  double _oldTime;
  double _kDistEpsilon = 2;
  private double _gyroAngle;
  PidConstants constants = new PidConstants(0.2, 0, 0);
  PidController controller = new PidController(constants) ;


  public LineDrive(Vector2 vector, boolean isFieldOriented) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(_drive);
    _vector = vector;
    _isFieldOriented = isFieldOriented;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _oldTime = Timer.getFPGATimestamp();
    Vector2 startVec = _drive.getKinematicPosition();
    _targetVec = getFinalPos(startVec, _vector, _isFieldOriented);
    _vector = _vector.scale(1/_vector.length);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double length = getDist();
    _time = Timer.getFPGATimestamp() - _oldTime;
    _oldTime = Timer.getFPGATimestamp();
    _drive.holonomicDrive(_vector.scale(-controller.calculate(_vector.length, _time)), 0, _isFieldOriented);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return util.epsilonEquals(getDist() , _kDistEpsilon);
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

  private Vector2 getFinalPos(Vector2 startPos, Vector2 add, boolean isFieldOriented){
    if(isFieldOriented){
      return startPos.add(add);
    } else{
      return startPos.add(add.rotateBy(_drive.getGyroscope().getAngle()));
    }
  }

  private double getDist(){
    return _drive.getKinematicPosition().subtract(_targetVec).length;
  }
}
