/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.util;

/**
 * Add your docs here.
 */
public class util {

    public static boolean epsilonEquals(double one, double two, double epsilon){
        return Math.abs(one - two) < epsilon;
    }

    public static boolean epsilonEquals(double dub, double epsilon){
        return epsilonEquals(dub, 0, epsilon);
    }

    public static double deadband(double num, double db){
        if (Math.abs(num) < db){
            return 0;
        } else {
            return num;
        }
    }
}
