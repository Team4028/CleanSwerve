package com.swervedrivespecialties.exampleswerve.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.commands.auton.saaar;
import com.swervedrivespecialties.exampleswerve.commands.drive.DriveCommand;
import com.swervedrivespecialties.exampleswerve.util.VisionData;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Command;

import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModule;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.robot.subsystems.SwerveDrivetrain;

public class DrivetrainSubsystem extends SwerveDrivetrain {
    private static final double TRACKWIDTH = 21.5;
    private static final double WHEELBASE = 23.5;

    private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(357.07);//357.16);
    private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(359.11);//359.11);
    private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(239.379);//238.53);
    private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(143.707);//140.59);

    private static final Object INSTANCE_LOCK = new Object();
    private static DrivetrainSubsystem instance;

    private final SwerveModule[] swerveModules;

    private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);

    private boolean _isFieldOriented = true; //When the robot starts up, the drivetrain is field oriented.

    private double curMinSpeed = .25;

    private boolean isRunningSaaar = false;
    private double kRangingDistance = 6 * 12;
    private Supplier<VisionData> visionDataSupplier;


    public DrivetrainSubsystem() {
        gyroscope.calibrate();
        gyroscope.setInverted(true); // You might not need to invert the gyro

        SwerveModule frontLeftModule = new Mk2SwerveModule(
                new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                FRONT_LEFT_ANGLE_OFFSET,
                new Spark(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER)
        );
        frontLeftModule.setName("Front Left");

        SwerveModule frontRightModule = new Mk2SwerveModule(
                new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                FRONT_RIGHT_ANGLE_OFFSET,
                new Spark(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER)
        );
        frontRightModule.setName("Front Right");

        SwerveModule backLeftModule = new Mk2SwerveModule(
                new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                BACK_LEFT_ANGLE_OFFSET,
                new Spark(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER)
        );
        backLeftModule.setName("Back Left");

        SwerveModule backRightModule = new Mk2SwerveModule(
                new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                BACK_RIGHT_ANGLE_OFFSET,
                new Spark(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER)
        );
        backRightModule.setName("Back Right");

        swerveModules = new SwerveModule[]{
                frontLeftModule,
                frontRightModule,
                backLeftModule,
                backRightModule,
        };
    }

    public static DrivetrainSubsystem getInstance() {
        synchronized (INSTANCE_LOCK) {
            if (instance == null) {
                instance = new DrivetrainSubsystem();
            }

            return instance;
        }
    }

    @Override
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    @Override
    public Gyroscope getGyroscope() {
        return gyroscope;
    }

    @Override
    public double getMaximumVelocity() {
        return 0;
    }

    @Override
    public double getMaximumAcceleration() {
        return 0;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveCommand());
    }

    public void toggleFieldOriented(){
        _isFieldOriented = !_isFieldOriented;
    }

    public boolean getFieldOriented(){
        return _isFieldOriented;
    }

    public void toggleMinSpeed(){
        if (curMinSpeed == 1.){
            curMinSpeed = .25;
        } else {
            curMinSpeed += .25;
        }
    }

    public void resetMinSpeed(){
        curMinSpeed = .25;
    }

    public double getMinSpeed(){
        return curMinSpeed;
    }

    public boolean getIsRunningSaaar(){
        return isRunningSaaar;
    }

    public void setIsRunningSaaar(boolean isRunning){
        isRunningSaaar = isRunning;
    }

    public void toggleAimAndRange(){
        if (!isRunningSaaar){
            isRunningSaaar = true;
            Command cmd = new saaar(kRangingDistance, visionDataSupplier);
            cmd.start();
        } else {
            isRunningSaaar = false;
        }
    }

    public double getGyroAngleDegrees(){
        return this.getGyroscope().getAngle().toDegrees();
    }
}
