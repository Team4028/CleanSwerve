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
  double kLowTransPassAlpha = .97;
  double lastTransPhi;
  double kTransF = 0.075;

  PidConstants transPidConstants = new PidConstants(.005, 0., 0.); //.005
  PidConstants rotPidConstants = new PidConstants(.012, 0., 0.);
  
  PidController transPidController = new PidController(transPidConstants);
  PidController rotPidController = new PidController(rotPidConstants);

  boolean hasFirstDriveVec;

  Vector2 driveVec;
  double l;
  double phi;

  double kDistanceEpsilon = 11.5; 
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
    l = 0;
    phi = 0;
    driveVec = Vector2.ZERO;
    hasFirstDriveVec = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    updateTime();
    if (_limelight.getHasTarget()){
      if (!hasFirstDriveVec){
        hasFirstDriveVec = true;
      }
      l = _limelight.getDistanceToTarget(Target.POWERCELL);
      phi = _limelight.getAngle1();
      driveVec = Vector2.fromAngle(Rotation2.fromDegrees(phi)).scale(l);
    } else {
      if (!hasFirstDriveVec){
        l = 0;
        phi = 0;
        driveVec = Vector2.ZERO;
      } else {
        double lastTarg = Math.atan2(driveVec.y, driveVec.x);
        driveVec = driveVec.subtract(_drive.getKinematicVelocity().rotateBy(_drive.getGyroscope().getAngle().inverse()).scale(dTime));
        double curTarg = Math.atan2(driveVec.y, driveVec.x);
        double deltaTarg = curTarg - lastTarg;
        l = driveVec.length;
        phi += _drive.getGyroscope().getRate() * dTime - deltaTarg;
      }
    }
    System.out.println("Length: " + l);
    double transMult = transPidController.calculate(l, dTime) + Math.copySign(kTransF, transPidController.calculate(l, dTime));
    double angMult = getRotationCmd(phi);
    _drive.holonomicDrive(Vector2.fromAngle(Rotation2.fromDegrees(getTransPhi(phi))).scale(-transMult), angMult, false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return util.epsilonEquals(l, kDistanceEpsilon) && util.epsilonEquals(phi, kAngleEpsilon) && hasFirstDriveVec;
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
}
