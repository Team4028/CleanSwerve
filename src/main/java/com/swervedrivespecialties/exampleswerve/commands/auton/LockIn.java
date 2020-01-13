/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.auton;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight.Target;
import com.swervedrivespecialties.exampleswerve.util.util;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class LockIn extends Command {
  DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();
  Limelight _ll = Limelight.getInstance();

  PidConstants findConstants = new PidConstants(.0005, 0, .0005);
  PidController findController = new PidController(findConstants);
  double findFF = .05;
  double kFindingMaxPhi = 5; //degrees

  PidConstants holdConstants = new PidConstants(.0007, 0, .0006);
  PidController holdController = new PidController(holdConstants);
  double kRotFeedForwardVelo = (3600 / 14.3) / .5; 
  double kRotFeedForwardStatic = .015;

  double kEpsilonT = .005;

  double lastTime;
  double dTime;

  double setpt;

  Vector2 curPos;

  public LockIn() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(_drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    holdController.setContinuous(true);
    findController.setContinuous(true);
    setpt = _drive.getGyroAngleDegrees() - _ll.getAngle1();
    Vector2 curPosWithTargZero = Vector2.fromAngle(Rotation2.fromDegrees(-setpt)).scale(_ll.getDistanceToTarget(Target.OLD));
    _drive.resetKinematics(curPosWithTargZero, Timer.getFPGATimestamp());
    findController.setSetpoint(0);
    holdController.setSetpoint(0);
    lastTime = Timer.getFPGATimestamp();
    curPos = _drive.getKinematicPosition();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    updateTime();
    double err = setpt - _drive.getGyroAngleDegrees();
    double forward = -Robot.getOi().getRawForwadCmd();
    forward = Math.copySign(Math.pow(forward, 2.0), forward) * .25;
    double strafe = -Robot.getOi().getRawStrafeCmd();
    strafe = Math.copySign(Math.pow(strafe, 2.0), strafe) * .25;
    double cmd;
    if (Math.abs(err) > kFindingMaxPhi){
      cmd = Math.copySign(findController.calculate(setpt, dTime), err);
    } else {
      Vector2 veloVec = new Vector2(forward, strafe);
      veloVec = veloVec.rotateBy(Rotation2.fromDegrees(90));
      Vector2 toTarget = _drive.getKinematicPosition().scale(-1);
      double scale = -toTarget.dot(veloVec) / Math.pow(toTarget.length, 2); //approximates length as unchanging
      cmd = toTarget.length > .1 ? -scale * 15 : 0; //ffVRot * err < 0 ? util.signum(ffVRot) * (Math.abs(ffVRot) + kRotFeedForwardStatic) : 0;
      cmd +=  Math.copySign(holdController.calculate(err,dTime), err);
      System.out.println("CMD: " + cmd);
    }
    _drive.holonomicDrive(new Vector2(forward, strafe), cmd, true);
    System.out.println("Phi Dot: " + getPhiDot());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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

  private void updateTime(){
    dTime = Timer.getFPGATimestamp() - lastTime;
    lastTime += dTime;
  }

  private double getTargetAngle(Vector2 vec){
    return vec.scale(-1).getAngle().toDegrees();
  }

  private double getPhiDot(){
    Vector2 velo = DrivetrainSubsystem.getInstance().getKinematicVelocity();
    double phi_naught = getTargetAngle(curPos);
    double phi_one = getTargetAngle(curPos.add(velo.scale(kEpsilonT)));
    return (phi_one - phi_naught) / kEpsilonT;
  }
}
