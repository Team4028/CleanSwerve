/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.auton;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.util.util;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.common.math.Rotation2;

public class TranslateCommandLL extends Command {

  private NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry camtran = nt.getEntry("camtran");
  
  private double fTrans = .05;
  private double pTrans = 0.017;
  private double iTrans = 0;
  private double dTrans = 0;
  private double fRot = .03;
  private double pRot = 0.007;
  private double iRot = 0;
  private double dRot = 0.0014;
  private double llX;
  private double _llXOffset;
  private double targetDist;
  private int count = 0;
  private double _kRotEpsilon = 1;
  private double _kTranEpsilon = 1;


  private double _currentTime;
  private double _localTime;
  private double _deltaTime;

  private PidController _transPIDController = new PidController(new PidConstants(pTrans, iTrans, dTrans));
  private PidController _rotPIDController = new PidController(new PidConstants(pRot, iRot, dRot));

  private static double[] defaultValArray = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  private DrivetrainSubsystem _driveTrainSubsystem = DrivetrainSubsystem.getInstance();

  public TranslateCommandLL() {
    requires(_driveTrainSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _currentTime = Timer.getFPGATimestamp();
    llX = camtran.getDoubleArray(defaultValArray)[0];
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    _localTime = Timer.getFPGATimestamp();
    _deltaTime = _localTime - _currentTime;
    _currentTime = _localTime;
    double gyroYaw = _driveTrainSubsystem.getGyroscope().getAngle().toDegrees();
    double llYaw = camtran.getDoubleArray(defaultValArray)[4];
    _llXOffset = nt.getEntry("tx").getDouble(0);
    llX = camtran.getDoubleArray(defaultValArray)[0];
    double alpha = gyroYaw - llYaw;// + llXSign;
    if(llX != 0 || count == 4){
      targetDist = llX;
      count = 0;
    }
    else{
      count += 1;
    }
    Vector2 holonomicTranslationPIDCmd = Vector2.fromAngle(Rotation2.fromDegrees(alpha)).scale(fTrans + _transPIDController.calculate(targetDist, _deltaTime));

    _driveTrainSubsystem.holonomicDrive(holonomicTranslationPIDCmd, fRot + _rotPIDController.calculate(_llXOffset, _deltaTime));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return util.epsilonEquals(llX, _kTranEpsilon) && util.epsilonEquals(_llXOffset, _kRotEpsilon);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    _driveTrainSubsystem.holonomicDrive(Vector2.ZERO, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  public static double getSkew(){
    double raw =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
    return Math.PI*Math.min(-raw, 90 + raw);
  }
  public static double getTX(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }
  public static double getXOff(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(defaultValArray)[0];
  }
  public static double getLLYaw(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(defaultValArray)[4];
  }
}
