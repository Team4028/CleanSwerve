/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.util;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class VisionData {
    double angleOne;
    double distance;
    boolean seesTarget;

    public VisionData(double theta, double l, boolean has){
        angleOne = theta;
        distance = l;
        seesTarget = has;
    }

    public double getAngle(){
        return angleOne;
    }

    public double getDistance(){
        return distance;
    }

    public boolean getSeesTarget(){
        return seesTarget;
    }

    //Robot Oriented field vector to target
    public Vector2 getVec(){
        return Vector2.fromAngle(Rotation2.fromDegrees(angleOne)).scale(distance);
    }
}
