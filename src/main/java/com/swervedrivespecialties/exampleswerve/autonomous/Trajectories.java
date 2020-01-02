/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.autonomous;

import java.util.function.Supplier;

import org.frcteam2910.common.control.CentripetalAccelerationConstraint;
import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.PathArcSegment;
import org.frcteam2910.common.control.PathLineSegment;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class Trajectories {
    ////////// UNIVERSAL TRAJECTORY CONSTANTS /////////////
    private static final int kSubdivideIterations = 8;
    private static final double kDefaultMaxSpeed = 10 * 12;
    private static final double kMaxAccel = 11. * 12;
    private static final double kMaxCentripedalAccel = 25. * 12;

    ////////////// TEST TRAJECTORY ////////////////////////
    private static Trajectory testTrajectory;
    private static final double testTrajectoryMaxVelo = 8 * 12;
    private static final ITrajectoryConstraint[] testTrajectoryConstraints = {new MaxVelocityConstraint(testTrajectoryMaxVelo), 
                                                                             new MaxAccelerationConstraint(kMaxAccel), 
                                                                             new CentripetalAccelerationConstraint(kMaxCentripedalAccel)};
    private static final double testTrajectoryEndVelo = 3 * 12;
    private static final Rotation2 testPathRotation = Rotation2.fromDegrees(90);
    private static final Rotation2 testPathStartRotation = Rotation2.ZERO;

    private static void generateTestTrajectory(){
        Path testPath = new Path(testPathStartRotation);
        testPath.addSegment(new PathLineSegment(Vector2.ZERO, new Vector2(24, 0)), testPathRotation);
        testPath.addSegment(new PathArcSegment(new Vector2(24, 0), new Vector2(72, 48), new Vector2(24, 48)), testPathRotation);
        testPath.subdivide(kSubdivideIterations);
        testTrajectory = new Trajectory(0.0, testTrajectoryEndVelo, testPath, testTrajectoryConstraints);
    }

    private static Trajectory getTestTrajectory(){
        return testTrajectory;
    }

    public static Supplier<Trajectory> testTrajectorySupplier = () -> getTestTrajectory();
    ////////////////////////////////////////////////////////


    //////////////// EXAMPLE TRAJECTORY //////////////////
    private static Trajectory exampleTrajectory;
    private static final double exampleTrajectoryX = 42;
    private static final double exampleTrajectoryY = -24;
    private static final double exampleTrajectoryPhi = -60; //degrees

    private static void generateExampleTrajectory(){
        exampleTrajectory = generateLineTrajectory(new Vector2(exampleTrajectoryX, exampleTrajectoryY), Rotation2.fromDegrees(exampleTrajectoryPhi));
    }
    
    private static Trajectory getExampleTrajectory(){
        return exampleTrajectory;
    }

    public static Supplier<Trajectory> exampleTrajectorySupplier = () -> getExampleTrajectory();
    //////////////////////////////////////////////////////


    //////////////// TEST AUTO TRAJECTORY ONE //////////////////////////////////
    private static Trajectory testAutoTrajectoryOne;
    private static final double testAutoTrajectoryOneMaxVelo = 8 * 12;
    private static final ITrajectoryConstraint[] testAutoTrajectoryOneConstraints = {new MaxVelocityConstraint(testAutoTrajectoryOneMaxVelo), 
                                                                                      new MaxAccelerationConstraint(kMaxAccel), 
                                                                                      new CentripetalAccelerationConstraint(kMaxCentripedalAccel)};
    private static final double testAutoTrajectoryOneEndVelo = 3 * 12;
    private static final Rotation2 testAutoPathOneRotation = Rotation2.fromDegrees(90);
    private static final Rotation2 testAutoPathOneStartRotation = Rotation2.ZERO;

    private static void generateTestAutoTrajectoryOne(){
        Path testAutoPathOne = new Path(testAutoPathOneStartRotation);
        testAutoPathOne.addSegment(new PathLineSegment(Vector2.ZERO, new Vector2(24, 0)), testAutoPathOneRotation);
        testAutoPathOne.addSegment(new PathArcSegment(new Vector2(24, 0), new Vector2(72, 48), new Vector2(24, 48)), testAutoPathOneRotation);
        testAutoPathOne.subdivide(kSubdivideIterations);
        testTrajectory = new Trajectory(0.0, testAutoTrajectoryOneEndVelo, testAutoPathOne, testAutoTrajectoryOneConstraints);
    }

    private static Trajectory getTestAutoTrajectoryOne(){
        return testAutoTrajectoryOne;
    }

    public static Supplier<Trajectory> testAutoTrajectoryOneSupplier = () -> getTestAutoTrajectoryOne();
    //////////////////////////////////////////////////////////////////////////

    ///////////////// TEST AUTO TRAJECTORY TWO ///////////////////////////////
    static double testAutoTrajectoryTwoSpeed = 8 * 12;
    static Rotation2 testAutoTrajectoryTwoRotation = Rotation2.fromDegrees(111.1);
    private static Trajectory testAutoTrajectoryTwo;
    
    public static void generateTestAutoTrajectoryTwo(){
        testAutoTrajectoryTwo= generateLineTrajectory(new Vector2(50.9, -130.5), testAutoTrajectoryTwoSpeed, testAutoPathOneRotation, testAutoTrajectoryTwoRotation);
    }

    private static Trajectory getTestAutoTrajectoryTwo(){
        return testAutoTrajectoryTwo;
    }

    public static Supplier<Trajectory> testAutoTrajectoryTwoSupplier = () -> getTestAutoTrajectoryTwo();
    //////////////////////////////////////////////////////////////////////////

    //////////////// TEST AUTO TRAJECTORY THREE //////////////////////////////////
    private static Trajectory testAutoTrajectoryThree;
    private static final double testAutoTrajectoryThreeMaxVelo = 8 * 12;
    private static final ITrajectoryConstraint[] testAutoTrajectoryThreeConstraints = {new MaxVelocityConstraint(testAutoTrajectoryThreeMaxVelo), 
                                                                                      new MaxAccelerationConstraint(kMaxAccel), 
                                                                                      new CentripetalAccelerationConstraint(kMaxCentripedalAccel)};
    private static final double testAutoTrajectoryThreeEndVelo = 3 * 12;
    private static final Rotation2 testAutoPathThreeRotation = Rotation2.fromDegrees(270);
    private static final Rotation2 testAutoPathThreeStartRotation = testAutoTrajectoryTwoRotation;

    private static void generateTestAutoTrajectoryThree(){
        Path testAutoPathThree = new Path(testAutoPathThreeStartRotation);
        testAutoPathThree.addSegment(new PathArcSegment(Vector2.ZERO, new Vector2(-50.9, -60.9), new Vector2(0, -60.9)), testAutoPathThreeRotation);
        testAutoPathThree.subdivide(kSubdivideIterations);
        testTrajectory = new Trajectory(0.0, testAutoTrajectoryThreeEndVelo, testAutoPathThree, testAutoTrajectoryThreeConstraints);
    }

    private static Trajectory getTestAutoTrajectoryThree(){
        return testAutoTrajectoryThree;
    }

    public static Supplier<Trajectory> testAutoTrajectoryThreeSupplier = () -> getTestAutoTrajectoryThree();
    //////////////////////////////////////////////////////////////////////////

    ///////////////// TEST AUTO TRAJECTORY FOUR ///////////////////////////////
    static double testAutoTrajectoryFourSpeed = 8 * 12;
    static Rotation2 testAutoTrajectoryFourRotation = Rotation2.fromDegrees(90);
    private static Trajectory testAutoTrajectoryFour;
    
    public static void generateTestAutoTrajectoryFour(){
        testAutoTrajectoryFour = generateLineTrajectory(new Vector2(0, 47.4), testAutoTrajectoryFourSpeed, testAutoPathThreeRotation, testAutoTrajectoryFourRotation);
    }

    private static Trajectory getTestAutoTrajectoryFour(){
        return testAutoTrajectoryFour;
    }

    public static Supplier<Trajectory> testAutoTrajectoryFourSupplier = () -> getTestAutoTrajectoryFour();
    //////////////////////////////////////////////////////////////////////////

    ///////////////// TEST AUTO TRAJECTORY FIVE ///////////////////////////////
    static double testAutoTrajectoryFiveSpeed = 8 * 12;
    static Rotation2 testAutoTrajectoryFiveRotation = Rotation2.fromDegrees(90);
    private static Trajectory testAutoTrajectoryFive;
    
    public static void generateTestAutoTrajectoryFive(){
        testAutoTrajectoryFive = generateLineTrajectory(new Vector2(0, 144), testAutoTrajectoryFiveSpeed, testAutoTrajectoryFourRotation, testAutoTrajectoryFiveRotation);
    }

    private static Trajectory getTestAutoTrajectoryFive(){
        return testAutoTrajectoryFive;
    }

    public static Supplier<Trajectory> testAutoTrajectoryFiveSupplier = () -> getTestAutoTrajectoryFive();
    //////////////////////////////////////////////////////////////////////////



    public static void generateAllTrajectories(){
        generateTestAutoTrajectoryOne();
        generateTestAutoTrajectoryTwo();
        generateTestAutoTrajectoryThree();
        generateTestAutoTrajectoryFour();
        generateTestAutoTrajectoryFive();
        generateTestTrajectory();
        generateExampleTrajectory();
    }


    /////////////////// Helper Methods /////////////////////////
    public static Trajectory generateLineTrajectory(Vector2 line, Rotation2 endRotation){
        ITrajectoryConstraint[] lineTrajectoryConstraints = {new MaxVelocityConstraint(kDefaultMaxSpeed), 
                                                             new MaxAccelerationConstraint(kMaxAccel), 
                                                             new CentripetalAccelerationConstraint(kMaxCentripedalAccel)};
        Path linePath = new Path(Rotation2.ZERO);
        linePath.addSegment(new PathLineSegment(Vector2.ZERO, line), endRotation);
        linePath.subdivide(kSubdivideIterations);
        Trajectory resultTrajectory = new Trajectory(0.0, 0.0, linePath, lineTrajectoryConstraints);
        return resultTrajectory;
    }

    public static Trajectory generateLineTrajectory(Vector2 line, Rotation2 startRotation, Rotation2 endRotation){
        ITrajectoryConstraint[] lineTrajectoryConstraints = {new MaxVelocityConstraint(kDefaultMaxSpeed), 
                                                             new MaxAccelerationConstraint(kMaxAccel), 
                                                             new CentripetalAccelerationConstraint(kMaxCentripedalAccel)};
        Path linePath = new Path(startRotation);
        linePath.addSegment(new PathLineSegment(Vector2.ZERO, line), endRotation);
        linePath.subdivide(kSubdivideIterations);
        Trajectory resultTrajectory = new Trajectory(0.0, 0.0, linePath, lineTrajectoryConstraints);
        return resultTrajectory;
    }

    public static Trajectory generateLineTrajectory(Vector2 line, double speed,  Rotation2 startRotation, Rotation2 endRotation){
        ITrajectoryConstraint[] lineTrajectoryConstraints = {new MaxVelocityConstraint(speed), 
                                                             new MaxAccelerationConstraint(kMaxAccel), 
                                                             new CentripetalAccelerationConstraint(kMaxCentripedalAccel)};
        Path linePath = new Path(startRotation);
        linePath.addSegment(new PathLineSegment(Vector2.ZERO, line), endRotation);
        linePath.subdivide(kSubdivideIterations);
        Trajectory resultTrajectory = new Trajectory(0.0, 0.0, linePath, lineTrajectoryConstraints);
        return resultTrajectory;
    }
}
