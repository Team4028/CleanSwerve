/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.auton;

import java.util.function.Supplier;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight.Target;
import com.swervedrivespecialties.exampleswerve.util.VisionData;
import com.swervedrivespecialties.exampleswerve.util.util;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class DriveIn extends Command {
  DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();
  Limelight _limelight = Limelight.getInstance();

  double kLowRotPassAlpha = .5;
  double lastRotCmd;
  double kLowTransPassAlpha = .9;
  double lastTransPhi;

  PidConstants transPidConstants = new PidConstants(.005, 0., 0.);
  PidConstants rotPidConstants = new PidConstants(.015, 0., 0.);
  
  PidController transPidController = new PidController(transPidConstants);
  PidController rotPidController = new PidController(rotPidConstants);

  Vector2 lastDriveVec;
  boolean hasFirstDriveVec;

  Vector2 lastVec;
  Vector2 dVec;

  double lastPhi;
  double dPhi;

  double kDistanceEpsilon = .5; 
  double kAngleEpsilon = 1.;

  double lastTime;
  double dTime;

  public DriveIn() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(_drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    transPidController.setSetpoint(0);
    rotPidController.setSetpoint(0);
    _limelight.setPipeline(2.0);
    lastTime = Timer.getFPGATimestamp();
    lastRotCmd = 0;
    hasFirstDriveVec = false;
    lastVec = _drive.getKinematicPosition();
    lastPhi = _drive.getGyroscope().getAngle().toDegrees();
    dVec = Vector2.ZERO;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    updateTime();
    updateDVec();
    updatePhi();
    double l;
    double phi;
    if (_limelight.getHasTarget()){
      if (!hasFirstDriveVec){
        hasFirstDriveVec = true;
      }
      l = _limelight.getDistanceToTarget(Target.POWERCELL);
      phi = _limelight.getAngle1();
      lastDriveVec = Vector2.fromAngle(Rotation2.fromDegrees(phi)).scale(l);
    } else {
      Vector2 res = guessUpdateKinematics();
      if (res.length == 0){
        l = 0;
        phi = 0;
      } else {
        l = res.length;
        phi = lastPhi - dPhi;
      }
    }
    double transMult = transPidController.calculate(l, dTime);
    double angMult = getRotationCmd(phi);
    _drive.holonomicDrive(Vector2.fromAngle(Rotation2.fromDegrees(getTransPhi(phi))).scale(-transMult), angMult, false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return util.epsilonEquals(_limelight.getDistanceToTarget(Target.POWERCELL), kDistanceEpsilon) && util.epsilonEquals(_limelight.getAngle1(), kAngleEpsilon);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    _drive.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    _drive.stop();
  }

  private void updateTime(){
    dTime = Timer.getFPGATimestamp() - lastTime;
    lastTime += dTime;
  }

  private double applyRotLowPass(double thisRotCmd, double lastRotCmd){
    return kLowRotPassAlpha * thisRotCmd + (1 - kLowRotPassAlpha) * lastRotCmd;
  }
  private double applyTransLowPass(double thisTransPhi, double lastTransPhi){
    return kLowTransPassAlpha * thisTransPhi + (1 - kLowTransPassAlpha) * lastTransPhi;
  }

  private double getRotationCmd(double phi){
    double rawRotCmd = rotPidController.calculate(phi, dTime);
    lastRotCmd = applyRotLowPass(rawRotCmd, lastRotCmd);
    return lastRotCmd;
  }
  private double getTransPhi(double phi){
    double rawPhi = phi;
    lastTransPhi = applyTransLowPass(rawPhi, lastTransPhi);
    return lastTransPhi;
  }

  private void updateDVec(){
    dVec = _drive.getKinematicPosition().subtract(lastVec);
    lastDriveVec = lastVec.add(dVec);
  }

  private void updatePhi(){
    dPhi = _drive.getGyroscope().getAngle().toDegrees();
    lastPhi += dPhi;
  }

  private Vector2 guessUpdateKinematics(){
    if (!hasFirstDriveVec){
      lastDriveVec = lastDriveVec.subtract(dVec);
      return lastDriveVec;
    } else {
      return Vector2.ZERO;
    }
  }
}
