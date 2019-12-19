package com.swervedrivespecialties.exampleswerve.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.swervedrivespecialties.exampleswerve.RobotMap;

import org.frcteam2910.common.robot.subsystems.Subsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Shooter extends Subsystem{

    private Shooter(){}

    double kShooterADefaultVBus = .6;
    double kShooterBDefaultVBus = .6;
    double kFeederDefaultVBus = .3;

    boolean shouldRunShooter = false;
    boolean shouldRunFeeder = false;

    private static Shooter _instance = new Shooter();

    DigitalInput infeedLimitSwitch = new DigitalInput(0);

    public static Shooter getInstance(){
        return _instance;
    }

    TalonSRX _shooterTalonA = new TalonSRX(RobotMap.SHOOTER_TALON_A);
    TalonSRX _shooterTalonB = new TalonSRX(RobotMap.SHOOTER_TALON_B);
    TalonSRX _feederTalon = new TalonSRX(RobotMap.FEEDER_TALON);
    TalonSRX _infeedTalon = new TalonSRX(RobotMap.INFEED_TALON);

    DoubleSolenoid _puncher = new DoubleSolenoid(0, 1);
    DoubleSolenoid _succ = new DoubleSolenoid(2, 3);
    Value kPuncherDefaultValue = Value.kReverse;
    Value kSuccDefaultValue = Value.kForward;

    public void runShooter(boolean shouldRun){
        if (shouldRun){
            _shooterTalonA.set(ControlMode.PercentOutput, kShooterADefaultVBus);
            _shooterTalonB.set(ControlMode.PercentOutput, kShooterBDefaultVBus);
        } else {
            _shooterTalonA.set(ControlMode.PercentOutput, 0);
            _shooterTalonB.set(ControlMode.PercentOutput, 0);
        }
    }

    public void runFeeder(boolean shouldRun){
        if (shouldRun){
            _feederTalon.set(ControlMode.PercentOutput, kFeederDefaultVBus);
        } else {
            _feederTalon.set(ControlMode.PercentOutput, 0);
        }
    }

    public void runInfeed(double vbus){
        if (infeedLimitSwitch.get()){
            _infeedTalon.set(ControlMode.PercentOutput, Math.min(0, vbus));
        } else {
            _infeedTalon.set(ControlMode.PercentOutput, vbus);
        }
    }

	@Override
	public void outputToSmartDashboard() {
	}

    @Override
    public void stop() {
        runShooter(false);
        runFeeder(false);
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    protected void initDefaultCommand() {
    }

    public void reset(){
        shouldRunShooter = false;
        shouldRunFeeder = false;
    }

    public boolean getShouldRunShooter(){
        return shouldRunShooter;
    }

    public boolean getShouldRunFeeder(){
        return shouldRunFeeder;
    }

    public void setShouldRunShooter(boolean shouldRun){
        shouldRunShooter = shouldRun;
    }

    public void setShouldRunFeeder(boolean shouldRun){
        shouldRunFeeder = shouldRun;
    }

    public void togglePuncher(){
        if (_puncher.get() == Value.kForward){
            _puncher.set(Value.kReverse);
        } else if (_puncher.get() == Value.kReverse){
            _puncher.set(Value.kForward);
        } else{
            _puncher.set(kPuncherDefaultValue);
        }
    }

    public void resetPuncher(){
        _puncher.set(kPuncherDefaultValue);
    }

    public void toggleSucc(){
        if (_succ.get() == Value.kForward){
            _succ.set(Value.kReverse);
        } else if (_puncher.get() == Value.kReverse){
            _succ.set(Value.kForward);
        } else{
            _succ.set(kSuccDefaultValue);
        }
    }

    public void resetSucc(){
        _succ.set(kSuccDefaultValue);
    }

    public boolean getSwitch(){
        return infeedLimitSwitch.get();
    }
}
