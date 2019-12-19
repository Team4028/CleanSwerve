package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.autonomous.Trajectories;
import com.swervedrivespecialties.exampleswerve.commands.RunShooter;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frcteam2910.common.robot.subsystems.SubsystemManager;

public class Robot extends TimedRobot {
    /**
     * How often the control thread should run in seconds.
     * By default it runs every 5 milliseconds.
     */
    private static final double UPDATE_DT = 5.0e-3;

    private static final OI oi = new OI();

    private final SubsystemManager subsystemManager = new SubsystemManager(
            DrivetrainSubsystem.getInstance()
    );

    public static OI getOi() {
        return oi;
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
        Command runShooter = new RunShooter();
        runShooter.start();
        DrivetrainSubsystem.getInstance().resetMinSpeed();
    }

    @Override
    public void teleopPeriodic() {
        super.teleopPeriodic();
        System.out.println(DrivetrainSubsystem.getInstance().getMinSpeed());
        SmartDashboard.putBoolean("Limit Switch", Shooter.getInstance().getSwitch());
    }

    @Override
    public void robotInit() {
        subsystemManager.enableKinematicLoop(UPDATE_DT);
        Trajectories.generateAllTrajectories();
    }

    @Override
    public void robotPeriodic() {
        subsystemManager.outputToSmartDashboard();
        Scheduler.getInstance().run();
    }
}
