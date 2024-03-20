package frc.robot.maps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.digital.CSDigitalInput;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule.Configuration;
import com.chopshop166.chopshoplib.leds.ColorFormat;
import com.chopshop166.chopshoplib.leds.SegmentConfig;
import com.chopshop166.chopshoplib.maps.LedMap;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.motors.CSSpark;
import com.chopshop166.chopshoplib.motors.CSSparkFlex;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.motors.SmartMotorControllerGroup;
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
import frc.robot.maps.subsystems.UndertakerMap;

@RobotMapFor("00:80:2F:36:7C:49")
public class Vibrato extends RobotMap {

    private static void setStatusPeriods(CSSpark motor, int status0, int status1, int status2) {
        motor.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus0, status0);
        motor.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus1, status1);
        motor.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus2, status2);
    }

    @Override
    public SwerveDriveMap getDriveMap() {

        // Remember to divide by 360
        final double FLOFFSET = -0.429688;
        final double FROFFSET = 0.099121;
        final double RLOFFSET = -0.226562;
        final double RROFFSET = 0.477539;

        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot
        final double MODULE_OFFSET_XY = Units.inchesToMeters(10.875);
        final PigeonGyro2 pigeonGyro2 = new PigeonGyro2(1);

        final CSSparkMax frontLeftSteer = new CSSparkMax(4);
        final CSSparkMax frontRightSteer = new CSSparkMax(8);
        final CSSparkMax rearLeftSteer = new CSSparkMax(2);
        final CSSparkMax rearRightSteer = new CSSparkMax(6);

        CSSparkFlex frontLeftDrive = new CSSparkFlex(3);
        CSSparkFlex frontRightDrive = new CSSparkFlex(7);
        CSSparkFlex rearLeftDrive = new CSSparkFlex(1);
        CSSparkFlex rearRightDrive = new CSSparkFlex(5);

        setStatusPeriods(frontLeftSteer, 100, 100, 100);
        setStatusPeriods(frontRightSteer, 100, 100, 100);
        setStatusPeriods(rearLeftSteer, 100, 100, 100);
        setStatusPeriods(rearRightSteer, 100, 100, 100);

        setStatusPeriods(frontLeftDrive, 100, 10, 10);
        setStatusPeriods(frontRightDrive, 100, 10, 10);
        setStatusPeriods(rearLeftDrive, 100, 10, 10);
        setStatusPeriods(rearRightDrive, 100, 10, 10);

        frontLeftSteer.getMotorController().setInverted(true);
        frontRightSteer.getMotorController().setInverted(true);
        rearLeftSteer.getMotorController().setInverted(true);
        rearRightSteer.getMotorController().setInverted(true);

        frontLeftDrive.setInverted(true);
        frontRightDrive.setInverted(false);
        rearLeftDrive.setInverted(true);
        rearRightDrive.setInverted(false);

        frontLeftSteer.getMotorController().setSmartCurrentLimit(30);
        frontRightSteer.getMotorController().setSmartCurrentLimit(30);
        rearLeftSteer.getMotorController().setSmartCurrentLimit(30);
        rearRightSteer.getMotorController().setSmartCurrentLimit(30);

        // Configuration for MK4i with L2 speeds
        Configuration MK4i_L2 = new Configuration(SDSSwerveModule.MK4_V2.gearRatio,
                SDSSwerveModule.MK4_V2.wheelDiameter, new PIDValues(0.011, 0.00, 0.0002),
                new PIDValues(0.001, 0.0, 0.0, 0.192));

        // All Distances are in Meters
        // Front Left Module
        final CANcoder encoderFL = new CANcoder(2);
        CANcoderConfiguration encoderFLConfig = new CANcoderConfiguration();
        encoderFLConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderFLConfig.MagnetSensor.MagnetOffset = FLOFFSET;
        encoderFL.getConfigurator().apply(encoderFLConfig);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new CtreEncoder(encoderFL), frontLeftSteer, frontLeftDrive, MK4i_L2);

        // Front Right Module
        final CANcoder encoderFR = new CANcoder(4);
        CANcoderConfiguration encoderFRConfig = new CANcoderConfiguration();
        encoderFRConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderFRConfig.MagnetSensor.MagnetOffset = FROFFSET;
        encoderFR.getConfigurator().apply(encoderFRConfig);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new CtreEncoder(encoderFR), frontRightSteer, frontRightDrive, MK4i_L2);

        // Rear Left Module
        final CANcoder encoderRL = new CANcoder(1);
        CANcoderConfiguration encoderRLConfig = new CANcoderConfiguration();
        encoderRLConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderRLConfig.MagnetSensor.MagnetOffset = RLOFFSET;
        encoderRL.getConfigurator().apply(encoderRLConfig);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new CtreEncoder(encoderRL), rearLeftSteer, rearLeftDrive, MK4i_L2);

        // Rear Right Module
        final CANcoder encoderRR = new CANcoder(3);
        CANcoderConfiguration encoderRRConfig = new CANcoderConfiguration();
        encoderRRConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderRRConfig.MagnetSensor.MagnetOffset = RROFFSET;
        encoderRR.getConfigurator().apply(encoderRRConfig);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new CtreEncoder(encoderRR), rearRightSteer, rearRightDrive, MK4i_L2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(15);

        final double maxRotationRadianPerSecond = Math.PI * 2;

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
        setStatusPeriods(leftMotor, 10, 100, 100);
        setStatusPeriods(rightMotor, 10, 100, 100);

        rightMotor.getMotorController().follow(leftMotor.getMotorController(), true);
        leftMotor.getMotorController().setInverted(false);
        leftMotor.getMotorController().setIdleMode(IdleMode.kBrake);
        leftMotor.getMotorController().setSmartCurrentLimit(40);
        rightMotor.getMotorController().setIdleMode(IdleMode.kBrake);
        rightMotor.getMotorController().setSmartCurrentLimit(40);
        CSEncoder encoder = new CSEncoder(2, 3, false);
        encoder.setDistancePerPulse(360.0 / 2048.0);
        CSDutyCycleEncoderLocal absEncoder = new CSDutyCycleEncoderLocal(1);
        absEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
        absEncoder.setDistancePerRotation(360);
        // Adjust this to fix absolute encoder angle. If at your zero angle,
        // just put in that number, no need to make it negative
        absEncoder.setPositionOffset(60.2);
        CSFusedEncoder fusedEncoder = new CSFusedEncoder(encoder, absEncoder);
        ProfiledPIDController pid = new ProfiledPIDController(0.02, 0.0, 0.0, new Constraints(120, 500));
        pid.setTolerance(2);
        ArmFeedforward feedForward = new ArmFeedforward(0, 0.02, 0.37, 0);

        return new ArmRotateMap(new SmartMotorControllerGroup(leftMotor, rightMotor),
                pid, feedForward, fusedEncoder,
                // Hard limits
                new ValueRange(-14, 87),
                // Soft limits
                new ValueRange(0, 73),
                new ArmRotateMap.ArmPresetValues(-12, 87, 23, 13.5, -3,
                        -14));
    }

    @Override
    public IntakeMap getIntakeMap() {
        CSSparkMax topRoller = new CSSparkMax(12);
        setStatusPeriods(topRoller, 100, 100, 100);
        topRoller.getMotorController().setInverted(true);
        topRoller.getMotorController().setIdleMode(IdleMode.kBrake);
        topRoller.getMotorController().setSmartCurrentLimit(30);
        CSDigitalInput sensor = new CSDigitalInput(0);
        return new IntakeMap(topRoller, sensor::get, 0.3);
    }

    @Override
    public ShooterMap getShooterMap() {
        CSSparkFlex rightWheels = new CSSparkFlex(11);
        CSSparkFlex leftWheels = new CSSparkFlex(10);

        setStatusPeriods(rightWheels, 100, 100, 100);
        setStatusPeriods(leftWheels, 100, 100, 100);

        rightWheels.getMotorController().setSmartCurrentLimit(50);
        leftWheels.getMotorController().setSmartCurrentLimit(50);

        rightWheels.setControlType(ControlType.kVelocity);
        rightWheels.getPidController().setP(0.001);
        rightWheels.getPidController().setI(0);
        rightWheels.getPidController().setD(0.0);
        rightWheels.getPidController().setFF(0.000182);
        rightWheels.getEncoder().getRaw().setMeasurementPeriod(10);

        leftWheels.setControlType(ControlType.kVelocity);
        leftWheels.getPidController().setP(0.001);
        leftWheels.getPidController().setI(0);
        leftWheels.getPidController().setD(0.0);
        leftWheels.getPidController().setFF(0.000174);
        leftWheels.getEncoder().getRaw().setMeasurementPeriod(10);
        leftWheels.getMotorController().setInverted(true);
        return new ShooterMap(rightWheels, leftWheels, true);
    }

    @Override
    public LedMap getLedMap() {
        var result = new LedMap(0, 34);
        var leds = result.ledBuffer;

        SegmentConfig backBottom = leds.segment(11).tags("Shooter", "Intake", "Alliance");
        SegmentConfig backTop = leds.segment(12).tags("ArmRotate", "Vision", "Auto", "Alliance");
        SegmentConfig slants = leds.segment(11, ColorFormat.GRB).tags("Shooter", "Intake", "Alliance");
        return result;
    }

    @Override
    public UndertakerMap getUndertakerMap() {
        CSSparkFlex topRoller = new CSSparkFlex(16);
        CSSparkFlex bottomRoller = new CSSparkFlex(15);

        setStatusPeriods(topRoller, 10, 100, 100);
        setStatusPeriods(bottomRoller, 100, 100, 100);

        topRoller.getMotorController().setInverted(false);
        topRoller.getMotorController().setIdleMode(IdleMode.kCoast);
        topRoller.getMotorController().setSmartCurrentLimit(50);
        bottomRoller.getMotorController().setSmartCurrentLimit(40);
        bottomRoller.getMotorController().follow(topRoller.getMotorController(),
                false);
        return new UndertakerMap(new SmartMotorControllerGroup(topRoller,
                bottomRoller));
    }

    @Override
    public void setupLogging() {
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.recordMetadata("RobotMap", this.getClass().getSimpleName());
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    }
}
