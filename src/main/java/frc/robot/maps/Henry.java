package frc.robot.maps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.digital.CSDigitalInput;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule.Configuration;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.sensors.CSDutyCycleEncoder;
import com.chopshop166.chopshoplib.sensors.CSEncoder;
import com.chopshop166.chopshoplib.sensors.CSFusedEncoder;
import com.chopshop166.chopshoplib.sensors.CtreEncoder;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro2;
import com.chopshop166.chopshoplib.states.PIDValues;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.IntakeMap;
import frc.robot.maps.subsystems.ShooterMap;

@RobotMapFor("00:80:2F:17:F7:AF")
public class Henry extends RobotMap {

    @Override
    public SwerveDriveMap getDriveMap() {

        final double FLOFFSET = -0.776;
        final double FROFFSET = -0.38;
        final double RLOFFSET = 0.448;
        final double RROFFSET = -0.891;
        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot

        final double MODULE_OFFSET_XY = Units.inchesToMeters(13.33); // Frostbites was 9.89
        final PigeonGyro2 pigeonGyro2 = new PigeonGyro2(1);

        final CSSparkMax frontLeftSteer = new CSSparkMax(4, MotorType.kBrushless);
        final CSSparkMax frontRightSteer = new CSSparkMax(8, MotorType.kBrushless);
        final CSSparkMax rearLeftSteer = new CSSparkMax(2, MotorType.kBrushless);
        final CSSparkMax rearRightSteer = new CSSparkMax(6, MotorType.kBrushless);

        frontLeftSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        frontRightSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        rearLeftSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        rearRightSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);

        frontLeftSteer.getMotorController().setInverted(true);
        frontRightSteer.getMotorController().setInverted(true);
        rearLeftSteer.getMotorController().setInverted(true);
        rearRightSteer.getMotorController().setInverted(true);

        // Configuration for MK4i with L2 speeds
        Configuration MK4i_L2 = new Configuration(SDSSwerveModule.MK4_V2.gearRatio,
                SDSSwerveModule.MK4_V2.wheelDiameter, new PIDValues(0.011, 0.00, 0.0002));

        // All Distances are in Meters
        // Front Left Module
        final CANcoder encoderFL = new CANcoder(2);
        CANcoderConfiguration encoderFLConfig = new CANcoderConfiguration();
        encoderFLConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderFLConfig.MagnetSensor.MagnetOffset = FLOFFSET;
        encoderFL.getConfigurator().apply(encoderFLConfig);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new CtreEncoder(encoderFL), frontLeftSteer, new CSSparkMax(3, MotorType.kBrushless),
                MK4i_L2);

        // Front Right Module
        final CANcoder encoderFR = new CANcoder(4);
        CANcoderConfiguration encoderFRConfig = new CANcoderConfiguration();
        encoderFRConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderFRConfig.MagnetSensor.MagnetOffset = FROFFSET;
        encoderFR.getConfigurator().apply(encoderFRConfig);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new CtreEncoder(encoderFR), frontRightSteer, new CSSparkMax(7,
                        MotorType.kBrushless),
                MK4i_L2);

        // Rear Left Module
        final CANcoder encoderRL = new CANcoder(3);
        CANcoderConfiguration encoderRLConfig = new CANcoderConfiguration();
        encoderRLConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderRLConfig.MagnetSensor.MagnetOffset = RLOFFSET;
        encoderRL.getConfigurator().apply(encoderRLConfig);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new CtreEncoder(encoderRL), rearLeftSteer, new CSSparkMax(1,
                        MotorType.kBrushless),
                MK4i_L2);

        // Rear Right Module
        final CANcoder encoderRR = new CANcoder(1);
        CANcoderConfiguration encoderRRConfig = new CANcoderConfiguration();
        encoderRRConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderRRConfig.MagnetSensor.MagnetOffset = RROFFSET;
        encoderRR.getConfigurator().apply(encoderRRConfig);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new CtreEncoder(encoderRR), rearRightSteer, new CSSparkMax(5,
                        MotorType.kBrushless),
                MK4i_L2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(12);

        final double maxRotationRadianPerSecond = Math.PI;

        return new SwerveDriveMap(frontLeft, frontRight, rearLeft, rearRight,
                maxDriveSpeedMetersPerSecond,
                maxRotationRadianPerSecond, pigeonGyro2);

    }

    // @Override
    // public ArmRotateMap getArmRotateMap() {
    // CSSparkMax leftMotor = new CSSparkMax(13, MotorType.kBrushless);
    // CSSparkMax rightMotor = new CSSparkMax(14, MotorType.kBrushless);
    // rightMotor.getMotorController().follow(leftMotor.getMotorController(), true);
    // leftMotor.getMotorController().setInverted(false);
    // leftMotor.getMotorController().setIdleMode(IdleMode.kBrake);
    // leftMotor.getMotorController().setSmartCurrentLimit(40);
    // rightMotor.getMotorController().setIdleMode(IdleMode.kBrake);
    // rightMotor.getMotorController().setSmartCurrentLimit(40);
    // CSEncoder encoder = new CSEncoder(6, 7, true);
    // encoder.setDistancePerPulse(360.0 / 2048.0 / 2);
    // CSDutyCycleEncoder absEncoder = new CSDutyCycleEncoder(8);
    // absEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    // absEncoder.setDistancePerRotation(-360 / 2);
    // // Adjust this to move the encoder zero point to the retracted position
    // absEncoder.setPositionOffset(0);
    // CSFusedEncoder fusedEncoder = new CSFusedEncoder(encoder, absEncoder);
    // ProfiledPIDController pid = new ProfiledPIDController(0.0, 0.0, 0.0, new
    // Constraints(45, 2025));
    // pid.setTolerance(0);
    // ArmFeedforward feedForward = new ArmFeedforward(0, 0.39, 3.74, 0);

    // return new ArmRotateMap(leftMotor, pid, feedForward, fusedEncoder, 100, 0,
    // 120, 0);
    // }

    @Override
    public ShooterMap getShooterMap() {
        CSSparkMax topWheels = new CSSparkMax(11, MotorType.kBrushless);
        CSSparkMax bottomWheels = new CSSparkMax(10, MotorType.kBrushless);
        bottomWheels.getMotorController().follow(topWheels.getMotorController(), false);
        topWheels.setControlType(ControlType.kVelocity);
        topWheels.getPidController().setP(0.0002);
        topWheels.getPidController().setI(0);
        topWheels.getPidController().setD(0);
        topWheels.getPidController().setFF(0.00018);
        return new ShooterMap(topWheels);
    }

    @Override
    public IntakeMap getIntakeMap() {
        CSSparkMax topRoller = new CSSparkMax(12, MotorType.kBrushless);
        topRoller.getMotorController().setInverted(true);
        topRoller.getMotorController().setIdleMode(IdleMode.kBrake);
        CSDigitalInput sensor = new CSDigitalInput(9);
        sensor.setInverted(true);
        return new IntakeMap(topRoller, sensor::get);
    }

    @Override
    public void setupLogging() {
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.recordMetadata("RobotMap", this.getClass().getSimpleName());
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    }
}
