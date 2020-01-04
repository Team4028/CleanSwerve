/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

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
  private double distance;
  private enum Target{
    POWERCELL, HIGH, LOW;
  }
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public double getAngle1(){
    return tx.getDouble(0);
  }

  public double getDistanceToPowerCell(Target obj){
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
    };
    return distance;
  }
}
