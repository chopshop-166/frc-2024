package frc.robot.maps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.digital.CSDigitalInput;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule.Configuration;
import com.chopshop166.chopshoplib.leds.SegmentConfig;
import com.chopshop166.chopshoplib.maps.LedMap;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.sensors.CSDutyCycleEncoderLocal;
import com.chopshop166.chopshoplib.sensors.CSEncoder;
import com.chopshop166.chopshoplib.sensors.CSFusedEncoder;
import com.chopshop166.chopshoplib.sensors.CtreEncoder;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro2;
import com.chopshop166.chopshoplib.states.PIDValues;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
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

    private static void setStatusPeriods(CSSparkMax motor) {
        motor.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        motor.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        motor.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
        motor.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
    }

    @Override
    public SwerveDriveMap getDriveMap() {

        final double FLOFFSET = -0.776;
        final double FROFFSET = -0.38;
        final double RLOFFSET = 0.448;
        final double RROFFSET = -0.891;
        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot

        final double MODULE_OFFSET_XY = Units.inchesToMeters(10.875); // Frostbites was 9.89
        final PigeonGyro2 pigeonGyro2 = new PigeonGyro2(1);

        final CSSparkMax frontLeftSteer = new CSSparkMax(4);
        final CSSparkMax frontRightSteer = new CSSparkMax(8);
        final CSSparkMax rearLeftSteer = new CSSparkMax(2);
        final CSSparkMax rearRightSteer = new CSSparkMax(6);

        setStatusPeriods(frontLeftSteer);
        setStatusPeriods(frontRightSteer);
        setStatusPeriods(rearLeftSteer);
        setStatusPeriods(rearRightSteer);

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
                new CtreEncoder(encoderFL), frontLeftSteer, new CSSparkMax(3), MK4i_L2);

        // Front Right Module
        final CANcoder encoderFR = new CANcoder(4);
        CANcoderConfiguration encoderFRConfig = new CANcoderConfiguration();
        encoderFRConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderFRConfig.MagnetSensor.MagnetOffset = FROFFSET;
        encoderFR.getConfigurator().apply(encoderFRConfig);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new CtreEncoder(encoderFR), frontRightSteer, new CSSparkMax(7), MK4i_L2);

        // Rear Left Module
        final CANcoder encoderRL = new CANcoder(3);
        CANcoderConfiguration encoderRLConfig = new CANcoderConfiguration();
        encoderRLConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderRLConfig.MagnetSensor.MagnetOffset = RLOFFSET;
        encoderRL.getConfigurator().apply(encoderRLConfig);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new CtreEncoder(encoderRL), rearLeftSteer, new CSSparkMax(1), MK4i_L2);

        // Rear Right Module
        final CANcoder encoderRR = new CANcoder(1);
        CANcoderConfiguration encoderRRConfig = new CANcoderConfiguration();
        encoderRRConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderRRConfig.MagnetSensor.MagnetOffset = RROFFSET;
        encoderRR.getConfigurator().apply(encoderRRConfig);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new CtreEncoder(encoderRR), rearRightSteer, new CSSparkMax(5), MK4i_L2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(15);

        final double maxRotationRadianPerSecond = Math.PI;

        final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
                // HolonomicPathFollowerConfig, this should likely live in your
                // Constants class
                new PIDConstants(2, 0.0, 0.05), // Translation PID constants (OFF_AXIS)
                new PIDConstants(1, 0.0, 0.0), // Rotation PID constants (OFF_AXIS)
                2.0, // Max module speed, in m/s
                0.3429,
                // Drive base radius (OFF_AXIS) in meters. Distance from robot center to
                // furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
        );

        return new SwerveDriveMap(frontLeft, frontRight, rearLeft, rearRight,
                maxDriveSpeedMetersPerSecond,
                maxRotationRadianPerSecond, pigeonGyro2,
                config);

    }

    @Override
    public ArmRotateMap getArmRotateMap() {
        CSSparkMax leftMotor = new CSSparkMax(13);
        CSSparkMax rightMotor = new CSSparkMax(14);
        setStatusPeriods(leftMotor);
        setStatusPeriods(rightMotor);
        rightMotor.getMotorController().follow(leftMotor.getMotorController(), true);
        leftMotor.getMotorController().setInverted(false);
        leftMotor.getMotorController().setIdleMode(IdleMode.kBrake);
        leftMotor.getMotorController().setSmartCurrentLimit(40);
        rightMotor.getMotorController().setIdleMode(IdleMode.kBrake);
        rightMotor.getMotorController().setSmartCurrentLimit(40);
        CSEncoder encoder = new CSEncoder(6, 7, false);
        encoder.setDistancePerPulse(360.0 / 2048.0 / 2);
        CSDutyCycleEncoderLocal absEncoder = new CSDutyCycleEncoderLocal(8);
        absEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
        absEncoder.setDistancePerRotation(-360 / 2);
        // Adjust this to move the encoder zero point to the retracted position (can't
        // be negative, do 360 - #)
        absEncoder.setPositionOffset(-179.5);
        CSFusedEncoder fusedEncoder = new CSFusedEncoder(encoder, absEncoder);
        ProfiledPIDController pid = new ProfiledPIDController(0.005, 0.0, 0.0, new Constraints(120, 500));
        pid.setTolerance(2);
        // Kv 3.74
        ArmFeedforward feedForward = new ArmFeedforward(0, 0.04, 0.3, 0);

        return new ArmRotateMap(leftMotor, pid, feedForward, fusedEncoder, new ValueRange(.5, 90),
                new ValueRange(20, 75), new ArmRotateMap.ArmPresetValues(.5, 90, 28, 28, 11.5, 75, 11.5));
    }

    @Override
    public ShooterMap getShooterMap() {
        CSSparkMax topWheels = new CSSparkMax(11);
        CSSparkMax bottomWheels = new CSSparkMax(10);
        setStatusPeriods(topWheels);
        setStatusPeriods(bottomWheels);
        topWheels.setControlType(ControlType.kVelocity);
        topWheels.getPidController().setP(0.000055);
        topWheels.getPidController().setI(0);
        topWheels.getPidController().setD(0.0);
        topWheels.getPidController().setFF(0.000185);
        bottomWheels.setControlType(ControlType.kVelocity);
        bottomWheels.getPidController().setP(0.000055);
        bottomWheels.getPidController().setI(0);
        bottomWheels.getPidController().setD(0.0);
        bottomWheels.getPidController().setFF(0.000185);
        return new ShooterMap(topWheels, bottomWheels, false);
    }

    @Override
    public IntakeMap getIntakeMap() {
        CSSparkMax topRoller = new CSSparkMax(12);
        setStatusPeriods(topRoller);
        topRoller.getMotorController().setInverted(true);
        topRoller.getMotorController().setIdleMode(IdleMode.kBrake);
        CSDigitalInput sensor = new CSDigitalInput(9);
        sensor.setInverted(true);
        return new IntakeMap(topRoller, sensor::get, 0.75);
    }

    @Override
    public LedMap getLedMap() {
        var result = new LedMap(0, 56);
        var leds = result.ledBuffer;

        SegmentConfig underglow = leds.segment(56).tags("underglow", "Shooter", "Arm Rotate", "Intake", "HP signal",
                "Vision", "Fire", "Auto", "Alliance");
        return result;
    }

    @Override
    public void setupLogging() {
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.recordMetadata("RobotMap", this.getClass().getSimpleName());
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    }
}
