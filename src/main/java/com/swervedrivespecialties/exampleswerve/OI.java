package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.commands.ZeroGyro;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class OI {
    /*
       Add your joysticks and buttons here
     */
    private Joystick primaryJoystick = new Joystick(0);

    public OI() {
        // Back button zeroes the drivetrain
        new JoystickButton(primaryJoystick, 7).whenPressed(new ZeroGyro());
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
}
