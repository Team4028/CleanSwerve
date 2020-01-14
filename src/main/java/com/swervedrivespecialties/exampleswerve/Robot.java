package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.commands.auton.LineDrive;
import com.swervedrivespecialties.exampleswerve.commands.auton.QuickMath;
import com.swervedrivespecialties.exampleswerve.autonomous.Trajectories;
import com.swervedrivespecialties.exampleswerve.commands.shooter.RunShooter;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight.Target;

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
    private static final Limelight _limelight = Limelight.getInstance();

    private final SubsystemManager subsystemManager = new SubsystemManager(
            DrivetrainSubsystem.getInstance()
    );

    public static OI getOi() {
        return oi;
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
        Shooter.getInstance().setShouldRunShooter(false);
        Command runShooter = new RunShooter();
        runShooter.start();
        DrivetrainSubsystem.getInstance().resetMinSpeed();
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
        // Command command = new QuickMath();
        // command.start();
    }

    @Override
    public void teleopPeriodic() {
        super.teleopPeriodic();
        SmartDashboard.putBoolean("Limit Switch", Shooter.getInstance().getSwitch());
        SmartDashboard.putNumber("LL X", _limelight.getAngle1());
        SmartDashboard.putNumber("ll distance", _limelight.getDistanceToTarget(Target.HIGH));
        SmartDashboard.putNumber("TA", _limelight.getTA());
        SmartDashboard.putNumber("TShort", _limelight.getBoxShortLength());
    }

    @Override
    public void robotInit() {
        subsystemManager.enableKinematicLoop(UPDATE_DT);
        Trajectories.generateAllTrajectories();
        _limelight.setPipeline(6.0);
    }

    @Override
    public void robotPeriodic() {
        subsystemManager.outputToSmartDashboard();
        Scheduler.getInstance().run();
        //SmartDashboard.putNumber("ll distance", _limelight.getDistanceToTarget(Target.HIGH));
    }
}
