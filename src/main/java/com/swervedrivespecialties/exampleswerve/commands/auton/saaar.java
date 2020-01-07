/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.auton;

import java.util.function.Supplier;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.util.VisionData;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class saaar extends Command {
  DrivetrainSubsystem drive = DrivetrainSubsystem.getInstance();
  Supplier<VisionData> vDataSupplier;
  VisionData vData;

  double dist;
  double ang;

  Vector2 curPos;
  Rotation2 curRot;

  double targetDist;

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

  double kStrafeSpeedScale = .5;

  public saaar(double targetDistance, Supplier<VisionData> visionDataSupplier) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(drive);
    targetDist = targetDistance;
    vDataSupplier = visionDataSupplier;    
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
    Vector2 strafeVec = curPos.rotateBy(Rotation2.fromDegrees(90)).scale(updateStrafe() / curPos.length);
    Vector2 driveCmd = vec2drive.add(strafeVec);
    double angCmd = rotPidController.calculate(ang, dTime);
    drive.holonomicDrive(driveCmd, angCmd, false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return drive.getIsRunningSaaar();
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
    curPos = curPos.add(dPos);
    curRot = curRot.rotateBy(curGyroAngle.rotateBy(lastGyroAngle.inverse()));
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

  private double updateStrafe(){
    double strafe = -Robot.getOi().getRawStrafeCmd();
    return  Math.copySign(Math.pow(strafe, 2.0), strafe) * kStrafeSpeedScale;
  }
}
