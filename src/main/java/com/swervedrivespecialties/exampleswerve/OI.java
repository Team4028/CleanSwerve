package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.commands.auton.LineDrive;
import com.swervedrivespecialties.exampleswerve.autonomous.Trajectories;
import com.swervedrivespecialties.exampleswerve.commands.auton.FollowTrajectory;
import com.swervedrivespecialties.exampleswerve.commands.drive.RotateToAngleTimed;
import com.swervedrivespecialties.exampleswerve.commands.drive.RotateToLLTargetTimed;
import com.swervedrivespecialties.exampleswerve.commands.shooter.RunFeeder;
import com.swervedrivespecialties.exampleswerve.commands.drive.ToggleFieldOriented;
import com.swervedrivespecialties.exampleswerve.commands.drive.ToggleMinSpeed;
import com.swervedrivespecialties.exampleswerve.commands.shooter.TogglePunches;
import com.swervedrivespecialties.exampleswerve.commands.shooter.ToggleRunShooter;
import com.swervedrivespecialties.exampleswerve.commands.shooter.ToggleSucc;
import com.swervedrivespecialties.exampleswerve.commands.auton.TranslateCommandLL;
import com.swervedrivespecialties.exampleswerve.commands.auton.autons.testAuto;
import com.swervedrivespecialties.exampleswerve.commands.drive.ZeroGyro;
import com.swervedrivespecialties.exampleswerve.util.BeakXboxController;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
    /*
       Add your joysticks and buttons here
     */

     BeakXboxController primaryJoystick = new BeakXboxController(0);
     BeakXboxController secondaryJoystick = new BeakXboxController(1);

    public OI() {
        // Back button zeroes the drivetrain
        primaryJoystick.y.whenPressed(new ToggleMinSpeed());
        primaryJoystick.back.whenPressed(new ZeroGyro());
        primaryJoystick.x.whenPressed(new RotateToLLTargetTimed(1.5));
        primaryJoystick.start.whenPressed(new ToggleFieldOriented());
        //primaryJoystick.lb.whenPressed(new FollowTrajectory(Trajectories.testTrajectorySupplier));
        primaryJoystick.lb.whenPressed(new testAuto());
        primaryJoystick.rb.whenPressed(new ToggleSucc());
        
        secondaryJoystick.a.whenPressed(new ToggleRunShooter());
        secondaryJoystick.b.whenPressed(new RunFeeder());
        secondaryJoystick.x.whenPressed(new TogglePunches());
        // secondaryJoystick.rb.whenPressed(new TranslateCommandLL());
        // secondaryJoystick.lb.whenPressed(new LineDrive(new Vector2(-48, 0), false, Rotation2.ZERO));

        primaryJoystick.dPad.up.whenPressed(new RotateToAngleTimed(0, 1.5));
		primaryJoystick.dPad.upLeft.whenPressed(new RotateToAngleTimed(45, 1.5));
		primaryJoystick.dPad.left.whenPressed(new RotateToAngleTimed(90, 1.5));
		primaryJoystick.dPad.downLeft.whenPressed(new RotateToAngleTimed(135, 1.5));
		primaryJoystick.dPad.down.whenPressed(new RotateToAngleTimed(180, 1.5));
		primaryJoystick.dPad.downRight.whenPressed(new RotateToAngleTimed(225, 1.5));
		primaryJoystick.dPad.right.whenPressed(new RotateToAngleTimed(270, 1.5));
		primaryJoystick.dPad.upRight.whenPressed(new RotateToAngleTimed(315, 1.5));
    }
    
    public Joystick getPrimaryJoystick() {
        return primaryJoystick;
    }

    public double getRawForwadCmd(){
        return primaryJoystick.getRawAxis(1);
    }

    public double getRawStrafeCmd(){
        return primaryJoystick.getRawAxis(0);
    }

    public double getRawRotationCmd(){
        return primaryJoystick.getRawAxis(4);
    }

    public double getRawSpeedScaleCmd(){
        return primaryJoystick.getRawAxis(3);
    }

    public double getSnapTurnCmd(){
        return primaryJoystick.getPOV();
    }

    public double getInfeedCmd(){
        double raw =  secondaryJoystick.getRawAxis(3) - secondaryJoystick.getRawAxis(2);
        return raw >= 0 ? raw * .5 : raw * .8;
    }
}
