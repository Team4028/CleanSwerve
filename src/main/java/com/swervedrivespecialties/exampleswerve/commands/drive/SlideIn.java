/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight.Target;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class SlideIn extends Command {
  DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();
  Limelight _limelight = Limelight.getInstance();

  double lastRotCmd;
  double lastTransPhi;
  double kTransF = 0.075;

  double kBallVecAlpha = .5;

  PidConstants transPidConstants = new PidConstants(.005, 0., 0.); //.005
  PidConstants rotPidConstants = new PidConstants(.012, 0., 0.);
  
  PidController transPidController = new PidController(transPidConstants);
  PidController rotPidController = new PidController(rotPidConstants);

  Vector2 ballVec;

  double lastTime;
  double dTime;

  public SlideIn() {
    requires(_drive);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    ballVec = Vector2.ZERO;
    transPidController.setContinuous(true);
    rotPidController.setContinuous(true);
    transPidController.setSetpoint(0);
    rotPidController.setSetpoint(0);
    lastTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    updateTime();
    updateBallVec();
    Vector2 vecToBall = ballVec.subtract(_drive.getKinematicPosition());
    if (ballVec.length != 0 && vecToBall.length != 0){
      double angToBall = Math.atan2(vecToBall.y, vecToBall.x) - _drive.getGyroscope().getAngle().toDegrees();
      double distToBall = vecToBall.length;
      double transMult = transPidController.calculate(distToBall, dTime);
      double rotMult = rotPidController.calculate(angToBall, dTime);
      _drive.holonomicDrive(vecToBall.scale(transMult / vecToBall.length), rotMult, true);
    } else {
      _drive.stop();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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

  private void updateBallVec(){
    if (_limelight.getHasTarget()){
      Vector2 curVecRobotOriented = Vector2.fromAngle(Rotation2.fromDegrees(_limelight.getAngle1())).scale(_limelight.getDistanceToTarget(Target.POWERCELL));
      Vector2 curGuess = _drive.getKinematicPosition().add(curVecRobotOriented.rotateBy(_drive.getGyroscope().getAngle()));
      ballVec = ballVec.scale(1 - kBallVecAlpha).add(curGuess.scale(kBallVecAlpha));
    }
  }
}
