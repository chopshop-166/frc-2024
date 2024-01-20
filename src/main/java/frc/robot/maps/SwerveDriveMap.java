package frc.robot.maps;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.drive.MockSwerveModule;
import com.chopshop166.chopshoplib.drive.SwerveModule;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.sensors.gyro.MockGyro;
import com.chopshop166.chopshoplib.sensors.gyro.SmartGyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A hardware map suitable for a swerve drive.
 *
 * All Distances are in Meters
 */
public record SwerveDriveMap(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule rearLeft,
        SwerveModule rearRight, double maxDriveSpeedMetersPerSecond,
        double maxRotationRadianPerSecond, SmartGyro gyro) implements LoggableMap<SwerveDriveMap.Data> {

    /** A distance to use for default values. */
    private static final double DEFAULT_DISTANCE_FROM_CENTER = 0.381;

    @Override
    public void updateData(Data data) {
        data.frontLeft.update(this.frontLeft);
        data.frontRight.update(this.frontRight);
        data.rearLeft.update(this.rearLeft);
        data.rearRight.update(this.rearRight);

        data.gyroYawPositionDegrees = this.gyro.getRotation2d();
    }

    /** Construct a map that uses mocks for everything. */
    public SwerveDriveMap() {
        this(
                // Front Left
                new MockSwerveModule(new Translation2d(DEFAULT_DISTANCE_FROM_CENTER,
                        DEFAULT_DISTANCE_FROM_CENTER)),
                // Front Right
                new MockSwerveModule(new Translation2d(DEFAULT_DISTANCE_FROM_CENTER,
                        -DEFAULT_DISTANCE_FROM_CENTER)),
                // Rear Left
                new MockSwerveModule(new Translation2d(-DEFAULT_DISTANCE_FROM_CENTER,
                        DEFAULT_DISTANCE_FROM_CENTER)),
                // Rear Right
                new MockSwerveModule(new Translation2d(-DEFAULT_DISTANCE_FROM_CENTER,
                        -DEFAULT_DISTANCE_FROM_CENTER)),
                // Max speed (m/s)
                2.0,
                // Max rotation (rad/s)
                Math.PI,
                // Gyro
                new MockGyro());
    }

    public static class Data implements LoggableInputs {
        public class SwerveModuleData {
            public String name;
            public double drivePositionMeters;
            public double velocityMetersPerSec;
            public double podAngle;
            public double[] driveCurrentAmps;
            public double[] driveTempC;
            public double[] steeringCurrentAmps;
            public double[] steeringTempC;
            public SwerveModuleState desiredState = new SwerveModuleState();
            public double steeringSetpoint; // Added during season
            public ChassisSpeeds speeds; // Added during season

            public SwerveModuleData(String name) {
                this.name = name;
            }

            public void toLog(LogTable table) {
                LogTable subTable = table.getSubtable(this.name);
                // Desired state (velocity/angle)
                subTable.put("DesiredVelocityMetresPerSec",
                        this.desiredState.speedMetersPerSecond);
                subTable.put("DesiredAngleDegrees", this.desiredState.angle.getDegrees());
                // actual state (Position/velocity/angle)
                subTable.put("DrivePositionMeters", this.drivePositionMeters);
                subTable.put("DriveVelocityMetersPerSec", this.velocityMetersPerSec);
                subTable.put("ABSDriveVelocityMetersPerSec", Math.abs(this.velocityMetersPerSec)); // Added during
                                                                                                   // season
                subTable.put("DriveAngleDegrees", this.podAngle);
                // Drive Motor params
                subTable.put("DriveCurrentAmps", this.driveCurrentAmps);
                subTable.put("DriveTempCelsius", this.driveTempC);
                // Steering Motor params
                subTable.put("SteeringCurrentAmps", this.steeringCurrentAmps);
                subTable.put("SteeringTempCelsius", this.steeringTempC);

                subTable.put("SteeringSetpoint", this.steeringSetpoint); // Added during season
                // subTable.put("ChassisSpeeds", this.speeds); (probably remove)

            }

            public void fromLog(LogTable table) {
                LogTable subTable = table.getSubtable(this.name);
                // Desired state (velocity/angle)
                this.desiredState.speedMetersPerSecond = subTable.get("DesiredVelocityMetresPerSec",
                        this.desiredState.speedMetersPerSecond);
                this.desiredState.angle = Rotation2d.fromDegrees(subTable.get("DesiredAngleDegrees",
                        this.desiredState.angle.getDegrees()));
                // actual state (Position/velocity/angle)
                this.drivePositionMeters = subTable.get("DrivePositionMeters",
                        this.drivePositionMeters);

                this.velocityMetersPerSec = subTable.get("DriveVelocityMetersPerSec", this.velocityMetersPerSec);
                this.podAngle = subTable.get("DriveAngleDegrees", this.podAngle);
                this.driveCurrentAmps = subTable.get("DriveCurrentAmps", this.driveCurrentAmps);
                this.driveTempC = subTable.get("DriveTempCelsius", this.driveTempC);
                this.steeringCurrentAmps = subTable.get("SteeringCurrentAmps", this.steeringCurrentAmps);
                this.steeringTempC = subTable.get("SteeringTempCelsius", this.steeringTempC);
                this.steeringSetpoint = subTable.get("SteeringSetpoint", this.steeringSetpoint); // Added during season
                // this.speeds = subTable.get("ChassisSpeeds", this.speeds); (probably remove)
            }

            public void update(SwerveModule module) {
                this.drivePositionMeters = module.getDistance();
                this.velocityMetersPerSec = module.getState().speedMetersPerSecond;
                this.podAngle = module.getAngle().getDegrees();
                this.driveCurrentAmps = module.getDriveMotor().getCurrentAmps();
                this.driveTempC = module.getDriveMotor().getTemperatureC();
                this.steeringCurrentAmps = module.getSteeringMotor().getCurrentAmps();
                this.steeringTempC = module.getSteeringMotor().getTemperatureC();
                module.setDesiredState(this.desiredState);
                this.steeringSetpoint = module.getSteeringMotor().get(); // Added during season
                // this.speeds = module.ChassisSpeeds.get(); (probably remove)
            }

            public SwerveModuleState getModuleState() {
                return new SwerveModuleState(velocityMetersPerSec, Rotation2d.fromDegrees(podAngle));
            } // Added during season ^

            public SwerveModulePosition getModulePosition() {
                return new SwerveModulePosition(drivePositionMeters, Rotation2d.fromDegrees(podAngle));
            }
        }

        public SwerveModuleData frontLeft = new SwerveModuleData("FrontLeft");
        public SwerveModuleData frontRight = new SwerveModuleData("FrontRight");
        public SwerveModuleData rearLeft = new SwerveModuleData("rearLeft");
        public SwerveModuleData rearRight = new SwerveModuleData("rearRight");

        public Rotation2d gyroYawPositionDegrees = Rotation2d.fromDegrees(0);

        @Override
        public void toLog(LogTable table) {
            frontLeft.toLog(table);
            frontRight.toLog(table);
            rearLeft.toLog(table);
            rearRight.toLog(table);

            table.put("GyroYawPositionDegrees", this.gyroYawPositionDegrees);
        }

        @Override
        public void fromLog(LogTable table) {
            frontLeft.fromLog(table);
            frontRight.fromLog(table);
            rearLeft.fromLog(table);
            rearRight.fromLog(table);

            this.gyroYawPositionDegrees = table.get("GyroYawPositionDegrees", this.gyroYawPositionDegrees);
        }

    }
}
