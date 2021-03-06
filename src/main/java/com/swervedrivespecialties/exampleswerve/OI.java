package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.autonomous.Trajectories;
import com.swervedrivespecialties.exampleswerve.commands.FollowTrajectory;
import com.swervedrivespecialties.exampleswerve.commands.RotateToAngleTimed;
import com.swervedrivespecialties.exampleswerve.commands.RotateToLLTargetTimed;
import com.swervedrivespecialties.exampleswerve.commands.ToggleFieldOriented;
import com.swervedrivespecialties.exampleswerve.commands.ToggleMinSpeed;
import com.swervedrivespecialties.exampleswerve.commands.ZeroGyro;
import com.swervedrivespecialties.exampleswerve.util.BeakXboxController;

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
        primaryJoystick.lb.whenPressed(new FollowTrajectory(Trajectories.testTrajectorySupplier));

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

    public boolean getTalonACmd(){
        return secondaryJoystick.getRawButton(1);
    }

    public boolean getTalonBCmd(){
        return secondaryJoystick.getRawButton(2);
    }

    public boolean getTalonCCmd(){
        return secondaryJoystick.getRawButton(3);
    }
}
