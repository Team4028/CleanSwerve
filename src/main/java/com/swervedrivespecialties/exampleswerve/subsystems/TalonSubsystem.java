package com.swervedrivespecialties.exampleswerve.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.commands.RunTalonSubsystem;

import org.frcteam2910.common.robot.subsystems.Subsystem;

import edu.wpi.first.wpilibj.Talon;

public class TalonSubsystem extends Subsystem{

    private TalonSubsystem(){}

    double kTalonADefaultVBUS = .6;
    double kTalonBDefaultVBUS = .6;
    double kTalonCDefaultVBUS = .6;

    private static TalonSubsystem _instance = new TalonSubsystem();

    public static TalonSubsystem getInstance(){
        return _instance;
    }

    TalonSRX _talonA = new TalonSRX(RobotMap.TALON_SUBSYSTEM_TALON_A);
    TalonSRX _talonB = new TalonSRX(RobotMap.TALON_SUBSYSTEM_TALON_B);
    TalonSRX _talonC = new TalonSRX(RobotMap.TALON_SUBSYSTEM_TALON_C);

    public void runTalonA(boolean shouldRun){
        if (shouldRun){
            System.out.println("What the Heck");
            _talonA.set(ControlMode.PercentOutput, kTalonADefaultVBUS);
        } else {
            _talonA.set(ControlMode.PercentOutput, 0);
        }
    }

    public void runTalonB(boolean shouldRun){
        if (shouldRun){
            _talonB.set(ControlMode.PercentOutput, kTalonBDefaultVBUS);
        } else {
            _talonB.set(ControlMode.PercentOutput, 0);
        }
    }

    public void runTalonC(boolean shouldRun){
        if (shouldRun){
            _talonC.set(ControlMode.PercentOutput, kTalonCDefaultVBUS);
        } else {
            _talonC.set(ControlMode.PercentOutput, 0);
        }
    }

	@Override
	public void outputToSmartDashboard() {
	}

    @Override
    public void stop() {
        runTalonA(false);
        runTalonB(false);
        runTalonC(false);
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    protected void initDefaultCommand() {
    }
}
