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

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class LockAndLoad extends Command {

  DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();
  Limelight _ll = Limelight.getInstance();

  //These are HOLD controllers. 
  PidConstants rangingConstants = new PidConstants(.004, 0, 0.0008);
  PidConstants aimingConstants = new PidConstants(.008, 0, .001);

  PidController rangeController = new PidController(rangingConstants);
  PidController aimController = new PidController(aimingConstants);
  
  // This is theoretically very important. The rate of change of our angle strafing in a circle is proportional to the arc length which each cycle is proportional to the strafe velocity, which in turn we approximate as approximately proportional to strafe cmd.
  double kRotFeedForward = -5.; 
  double kRotCurrentFeedForward;

  double epsilon = .02;

  double lastTime;
  double dTime;

  double kStrafeSpeedScale = .25;

  double kTargetDistance;

  public LockAndLoad() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(_drive);
    rangeController.setContinuous(true);
    aimController.setContinuous(true);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    kTargetDistance = _ll.getDistanceToTarget(Target.OLD);
    rangeController.setSetpoint(kTargetDistance);
    aimController.setSetpoint(0);
    kRotCurrentFeedForward = kRotFeedForward / kTargetDistance;
    lastTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  //all robot oriented
  @Override
  protected void execute() {
    updateTime();
    double phi = _ll.getAngle1();
    System.out.println("ANGLE: " + phi);
    double l = _ll.getDistanceToTarget(Target.OLD);
    Vector2 aimVec = Vector2.fromAngle(Rotation2.fromDegrees(phi));
    Vector2 strafeVec = aimVec.rotateBy(Rotation2.fromDegrees(90));
    double rangeMult = rangeController.calculate(l, dTime);
    double strafe = -Robot.getOi().getRawStrafeCmd();
    double strafeMult = Math.copySign(Math.pow(strafe, 2.0), strafe) * kStrafeSpeedScale;
    Vector2 driveVec = aimVec.scale(0).add(strafeVec.scale(strafeMult)); //-rangeMult
    double rotationCmd = strafeMult * kRotCurrentFeedForward + aimController.calculate(phi, dTime);
    _drive.holonomicDrive(driveVec, rotationCmd, false);
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
}
