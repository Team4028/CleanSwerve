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
  private NetworkTableEntry tshort = nt.getEntry("tshort");
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
  public double getTA(){
    return ta.getDouble(0);
  }
  public double getBoxShortLength(){
    return tshort.getDouble(0);
  }

  public double getDistanceToTarget(Target obj){
    Target target = obj;
    switch (target) {
      default: distance = 0;
      case POWERCELL:
        double hypot = 63.971 * Math.pow(ta.getDouble(0), -0.461);
        distance = hypot > 8.5 ? Math.sqrt(hypot * hypot - 8.5 * 8.5) : 0;
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

  public boolean getHasTarget(){
    return tv.getDouble(0.0) != 0.0;
  }
}
