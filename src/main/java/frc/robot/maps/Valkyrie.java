package frc.robot.maps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule.Configuration;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.sensors.CtreEncoder;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro;
import com.chopshop166.chopshoplib.states.PIDValues;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.maps.subsystems.IntakeMap;

@RobotMapFor("00:80:2F:19:7B:A3")
public class Valkyrie extends RobotMap {

    @Override
    public SwerveDriveMap getDriveMap() {

        final double MODULE_OFFSET_XY = Units.inchesToMeters(12.375);
        final PigeonGyro pigeonGyro = new PigeonGyro(new PigeonIMU(0));
        pigeonGyro.setInverted(true);

        final CSSparkMax frontLeftSteer = new CSSparkMax(4);
        final CSSparkMax frontRightSteer = new CSSparkMax(8);
        final CSSparkMax rearLeftSteer = new CSSparkMax(2);
        final CSSparkMax rearRightSteer = new CSSparkMax(6);

        frontLeftSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        frontRightSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        rearLeftSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        rearRightSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);

        frontLeftSteer.getMotorController().setInverted(false);
        frontRightSteer.getMotorController().setInverted(false);
        rearLeftSteer.getMotorController().setInverted(false);
        rearRightSteer.getMotorController().setInverted(false);

        // Configuration for MK4 with L2 speeds
        Configuration MK4_V2 = new Configuration(SDSSwerveModule.MK4_V2.gearRatio,
                SDSSwerveModule.MK4_V2.wheelDiameter, new PIDValues(0.004, 0.00, 0.0002));

        // All Distances are in Meters
        // Front Left Module
        final CANcoder encoderFL = new CANcoder(2);
        CANcoderConfiguration encoderFLConfig = new CANcoderConfiguration();
        encoderFLConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderFLConfig.MagnetSensor.MagnetOffset = 0.396;
        encoderFL.getConfigurator().apply(encoderFLConfig);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new CtreEncoder(encoderFL), frontLeftSteer, new CSSparkMax(3), MK4_V2);

        // Front Right Module
        final CANcoder encoderFR = new CANcoder(4);
        CANcoderConfiguration encoderFRConfig = new CANcoderConfiguration();
        encoderFRConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderFRConfig.MagnetSensor.MagnetOffset = 0.789;
        encoderFR.getConfigurator().apply(encoderFRConfig);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new CtreEncoder(encoderFR), frontRightSteer, new CSSparkMax(7), MK4_V2);

        // Rear Left Module
        final CANcoder encoderRL = new CANcoder(1);
        CANcoderConfiguration encoderRLConfig = new CANcoderConfiguration();
        encoderRLConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderRLConfig.MagnetSensor.MagnetOffset = 0.209;
        encoderRL.getConfigurator().apply(encoderRLConfig);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new CtreEncoder(encoderRL), rearLeftSteer, new CSSparkMax(1), MK4_V2);

        // Rear Right Module
        final CANcoder encoderRR = new CANcoder(3);
        CANcoderConfiguration encoderRRConfig = new CANcoderConfiguration();
        encoderRRConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderRRConfig.MagnetSensor.MagnetOffset = 0.918;
        encoderRR.getConfigurator().apply(encoderRRConfig);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new CtreEncoder(encoderRR), rearRightSteer, new CSSparkMax(5), MK4_V2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(12);

        final double maxRotationRadianPerSecond = Math.PI;

        final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
                // HolonomicPathFollowerConfig, this should likely live in your
                // Constants class
                new PIDConstants(0.2, 0.0, 0.05), // Translation PID constants (OFF_AXIS)
                new PIDConstants(0.001, 0.0, 0.0), // Rotation PID constants (OFF_AXIS)
                2.0, // Max module speed, in m/s
                0.3429,
                // Drive base radius (OFF_AXIS) in meters. Distance from robot center to
                // furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
        );

        return new SwerveDriveMap(frontLeft, frontRight, rearLeft, rearRight,
                maxDriveSpeedMetersPerSecond,
                maxRotationRadianPerSecond, pigeonGyro,
                config);

    }

    @Override
    public IntakeMap getIntakeMap() {
        CSSparkMax topRoller = new CSSparkMax(11);
        CSSparkMax bottomRoller = new CSSparkMax(9);
        bottomRoller.getMotorController().follow(topRoller.getMotorController(), true);
        return new IntakeMap(topRoller, () -> false, 0.5);
    }

    @Override
    public void setupLogging() {
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.recordMetadata("RobotMap", this.getClass().getSimpleName());
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    }
}
