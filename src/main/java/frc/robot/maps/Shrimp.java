package frc.robot.maps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule.Configuration;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.motors.CSSpark;
import com.chopshop166.chopshoplib.motors.CSSparkFlex;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.sensors.CtreEncoder;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro2;
import com.chopshop166.chopshoplib.states.PIDValues;
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

@RobotMapFor("Shrimp")
public class Shrimp extends RobotMap {

    private static void setStatusPeriods(CSSpark motor, int status0, int status1, int status2) {
        motor.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus0, status0);
        motor.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus1, status1);
        motor.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus2, status2);
    }

    @Override
    public SwerveDriveMap getDriveMap() {

        // Remember to divide by 360
        // CAN ID 2
        final double FLOFFSET = 0;
        // CAN ID 4
        final double FROFFSET = 0;
        // CAN ID 1
        final double RLOFFSET = 0;
        // CAN ID 3
        final double RROFFSET = 0;

        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot
        final double MODULE_OFFSET_XY = Units.inchesToMeters(6.00015);
        final PigeonGyro2 pigeonGyro2 = new PigeonGyro2(1);

        final CSSparkMax frontLeftSteer = new CSSparkMax(4);
        final CSSparkMax frontRightSteer = new CSSparkMax(8);
        final CSSparkMax rearLeftSteer = new CSSparkMax(2);
        final CSSparkMax rearRightSteer = new CSSparkMax(6);

        CSSparkMax frontLeftDrive = new CSSparkMax(3);
        CSSparkMax frontRightDrive = new CSSparkMax(7);
        CSSparkMax rearLeftDrive = new CSSparkMax(1);
        CSSparkMax rearRightDrive = new CSSparkMax(5);

        setStatusPeriods(frontLeftSteer, 100, 100, 100);
        setStatusPeriods(frontRightSteer, 100, 100, 100);
        setStatusPeriods(rearLeftSteer, 100, 100, 100);
        setStatusPeriods(rearRightSteer, 100, 100, 100);

        setStatusPeriods(frontLeftDrive, 100, 10, 10);
        setStatusPeriods(frontRightDrive, 100, 10, 10);
        setStatusPeriods(rearLeftDrive, 100, 10, 10);
        setStatusPeriods(rearRightDrive, 100, 10, 10);

        frontLeftSteer.getMotorController().setInverted(false);
        frontRightSteer.getMotorController().setInverted(false);
        rearLeftSteer.getMotorController().setInverted(false);
        rearRightSteer.getMotorController().setInverted(false);

        frontLeftDrive.setInverted(true);
        frontLeftDrive.getEncoder().getRaw().setMeasurementPeriod(8);
        frontLeftDrive.getEncoder().getRaw().setAverageDepth(2);
        frontRightDrive.setInverted(false);
        frontRightDrive.getEncoder().getRaw().setMeasurementPeriod(8);
        frontRightDrive.getEncoder().getRaw().setAverageDepth(2);
        rearLeftDrive.setInverted(true);
        rearLeftDrive.getEncoder().getRaw().setMeasurementPeriod(8);
        rearLeftDrive.getEncoder().getRaw().setAverageDepth(2);
        rearRightDrive.setInverted(false);
        rearRightDrive.getEncoder().getRaw().setMeasurementPeriod(8);
        rearRightDrive.getEncoder().getRaw().setAverageDepth(2);

        frontLeftSteer.getMotorController().setSmartCurrentLimit(30);
        frontRightSteer.getMotorController().setSmartCurrentLimit(30);
        rearLeftSteer.getMotorController().setSmartCurrentLimit(30);
        rearRightSteer.getMotorController().setSmartCurrentLimit(30);

        // Configuration for MK4i with L2 speeds
        Configuration MK4i_L2 = new Configuration(SDSSwerveModule.MK4_V2.gearRatio,
                SDSSwerveModule.MK4_V2.wheelDiameter, new PIDValues(0, 0, 0),
                new PIDValues(0, 0, 0, 0));

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

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(14.5);

        final double maxRotationRadianPerSecond = Math.PI * 2;

        final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
                // HolonomicPathFollowerConfig, this should likely live in your
                // Constants class
                new PIDConstants(0, 0, 0), // Translation PID constants (OFF_AXIS)
                new PIDConstants(0, 0.0, 0.0), // Rotation PID constants (OFF_AXIS)
                2.0, // Max module speed, in m/s
                0.2155317,
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
    public void setupLogging() {
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.recordMetadata("RobotMap", this.getClass().getSimpleName());
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    }
}
