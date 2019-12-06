package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.common.math.Vector2;

public class DriveCommand extends Command {

    public DriveCommand() {
        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void execute() {
        //All of the holonomic drive command, fwd, strafe, and rot are scaled by speedScale.
        double controllerSpeedScale = Robot.getOi().getRawSpeedScaleCmd();
        //square the speed scale trigger, and minimize it at .4
        controllerSpeedScale = .4 + .6 * controllerSpeedScale * controllerSpeedScale;

        double forward = -Robot.getOi().getRawForwadCmd();
        // Square the forward stick
        forward = Math.copySign(Math.pow(forward, 2.0), forward) * controllerSpeedScale;

        double strafe = -Robot.getOi().getRawStrafeCmd();
        // Square the strafe stick
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe) * controllerSpeedScale;

        double rotation = -Robot.getOi().getRawRotationCmd();
        // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation) * controllerSpeedScale;

        DrivetrainSubsystem.getInstance().holonomicDrive(new Vector2(forward, strafe), rotation, true);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
