/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import java.util.function.Supplier;

import com.swervedrivespecialties.exampleswerve.util.VisionData;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {
  private NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = nt.getEntry("tx");
  private NetworkTableEntry ta = nt.getEntry("ta");
  private NetworkTableEntry tv = nt.getEntry("tv");
  private NetworkTableEntry pipeline = nt.getEntry("pipeline");
  private double distance;
  public enum Target{
    POWERCELL, HIGH, LOW, OLD;
  }
  private static  Limelight _instance = new Limelight();
  public static Limelight getInstance(){
    return _instance;
  }
  private Limelight(){

  }
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public double getAngle1(){
    return tx.getDouble(0);
  }

  public double getDistanceToTarget(Target obj){
    Target target = obj;
    switch (target) {
      default: distance = 0;
      case POWERCELL:
        distance = 53.407 * Math.pow(ta.getDouble(0), 0.533);
        break;
      case HIGH:
        distance = 0;
        break;
      case LOW:
        distance = 0;
        break;
      case OLD:
        distance = 190.42 * Math.pow(ta.getDouble(0), -0.471);
        break;
    };
    return distance;
  }
  public void setPipeline(double pipe){
    pipeline.setDouble(pipe);
  }
  public Supplier<VisionData> getVDSupplier(){
    pipeline.setDouble(2.0);
    return () -> new VisionData(tx.getDouble(0), getDistanceToTarget(Target.POWERCELL), tv.getBoolean(false));
  } 
}
