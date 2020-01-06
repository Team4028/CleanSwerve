/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.auton;

import java.util.function.Supplier;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.util.VisionData;
import com.swervedrivespecialties.exampleswerve.util.util;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class DriveIn extends Command {
  DrivetrainSubsystem drive = new DrivetrainSubsystem();
  Supplier<VisionData> vDataSupplier;
  VisionData vData;

  double dist;
  double ang;

  Vector2 curPos;
  Rotation2 curRot;

  double lastTime;
  double dTime;
  Rotation2 curGyroAngle;
  Rotation2 lastGyroAngle;

  PidConstants transPidConstants = new PidConstants(.05, 0., 0.);
  PidConstants rotPidConstants = new PidConstants(.05, 0., 0.);
  
  PidController transPidController = new PidController(transPidConstants);
  PidController rotPidController = new PidController(rotPidConstants);

  double kDistanceEpsilon = .5; 
  double kAngleEpsilon = 1.;

  public DriveIn(Supplier<VisionData> vSupplier) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(drive);
    vDataSupplier = vSupplier;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    curGyroAngle = drive.getGyroscope().getAngle();
    lastTime = Timer.getFPGATimestamp();
    vData = vDataSupplier.get();
    curPos = vData.getVec();
    curRot = Rotation2.fromDegrees(vData.getAngle());
    transPidController.setSetpoint(0);
    rotPidController.setSetpoint(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    updateTime();
    updateGyroAngle();
    updateDriveVec();
    double transMult = transPidController.calculate(dist, dTime);
    Vector2 vec2drive = curPos.scale(transMult / curPos.length);  
    double angCmd = rotPidController.calculate(ang, dTime);
    drive.holonomicDrive(vec2drive, angCmd, false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return util.epsilonEquals(dist, kDistanceEpsilon) && util.epsilonEquals(ang, kAngleEpsilon);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drive.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    drive.stop();
  }

  private void guessUpdateKinematics(){
    Vector2 velo = drive.getKinematicVelocity().rotateBy(curGyroAngle);
    Vector2 dPos = velo.scale(dTime);
    curPos = curPos.subtract(dPos);
    curRot = curRot.rotateBy(curGyroAngle.rotateBy(lastGyroAngle.inverse())); //the parentheses could change here since SO(2) is abelian
    dist = curPos.length;
    ang = curRot.toDegrees();
  }

  private void updateTime(){
    dTime = Timer.getFPGATimestamp() - lastTime;
    lastTime += dTime;
  }

  private void updateGyroAngle(){
    lastGyroAngle = curGyroAngle;
    curGyroAngle = drive.getGyroscope().getAngle();
  }

  private void updateDriveVec(){
    vData = vDataSupplier.get();
    if (vData.getSeesTarget()){
      curPos = vData.getVec();
      ang = vData.getAngle();
      curRot = Rotation2.fromDegrees(ang);
      dist = curPos.length;
    } else {
      guessUpdateKinematics();
    }
  }
}
