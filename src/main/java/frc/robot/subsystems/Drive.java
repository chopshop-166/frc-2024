package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.race;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import com.chopshop166.chopshoplib.commands.FunctionalWaitCommand;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.CameraSwerveDriveMap;

public class Drive extends LoggedSubsystem<CameraSwerveDriveMap.Data, CameraSwerveDriveMap> {

    public final SwerveDriveKinematics kinematics;

    boolean isBlue = false;
    double maxDriveSpeedMetersPerSecond;
    double maxRotationRadiansPerSecond;
    double speedCoef = 1;
    double rotationCoef = 1;

    SwerveDrivePoseEstimator estimator;

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    private final SwerveDriveMap driveMap;

    public Drive(CameraSwerveDriveMap map) {

        super(new CameraSwerveDriveMap.Data(), map);

        this.driveMap = map.driveMap;

        driveMap.gyro.reset();
        kinematics = new SwerveDriveKinematics(
                driveMap.frontLeft.getLocation(),
                driveMap.frontRight.getLocation(),
                driveMap.rearLeft.getLocation(), driveMap.rearRight.getLocation());
        maxDriveSpeedMetersPerSecond = driveMap.maxDriveSpeedMetersPerSecond;
        maxRotationRadiansPerSecond = driveMap.maxRotationRadianPerSecond;

        estimator = new SwerveDrivePoseEstimator(kinematics,
                driveMap.gyro.getRotation2d(),
                getData().getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.02, 0.02, 0.01),
                VecBuilder.fill(0.1, 0.1, 0.01));

        AutoBuilder.configureHolonomic(estimator::getEstimatedPosition, this::setPose,
                this::getSpeeds, this::move, // Method that will drive the robot given ROBOT
                // RELATIVE ChassisSpeeds
                driveMap.pathFollower, () -> isBlue, this);
    }

    public void setPose(Pose2d pose) {
        estimator.resetPosition(driveMap.gyro.getRotation2d(),
                getData().getModulePositions(), pose);
    }

    public Command setPoseCommand(Pose2d pose) {
        return runOnce(() -> {
            setPose(pose);
        });
    }

    private void deadbandMove(final double xSpeed, final double ySpeed,
            final double rotation, boolean isRobotCentric) {

        var deadband = Modifier.scalingDeadband(DriverStation.isFMSAttached() ? 0.05 : 0.15);
        double rotationInput = deadband.applyAsDouble(rotation);
        double xInput = deadband.applyAsDouble(xSpeed);
        double yInput = deadband.applyAsDouble(ySpeed);

        final double translateXSpeed = xInput
                * maxDriveSpeedMetersPerSecond * speedCoef;
        final double translateYSpeed = yInput
                * maxDriveSpeedMetersPerSecond * speedCoef;
        final double rotationSpeed = rotationInput
                * maxRotationRadiansPerSecond * rotationCoef;
        move(translateXSpeed, translateYSpeed, rotationSpeed, isRobotCentric);
    }

    private void move(final double xSpeed, final double ySpeed,
            final double rotation, boolean isRobotCentric) {

        // rotationOffset is temporary and startingRotation is set at the start
        ChassisSpeeds speeds;
        if (isRobotCentric) {
            speeds = new ChassisSpeeds(ySpeed, xSpeed, rotation);
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, getData().gyroYawPosition);
        }

        move(speeds);

    }

    private void move(final ChassisSpeeds speeds) {

        // Now use this in our kinematics
        final SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxDriveSpeedMetersPerSecond);

        // All the states
        getData().setDesiredStates(moduleStates);
        Logger.recordOutput("Desired Module States", moduleStates);
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getData().getModuleStates());
    }

    // Yes! Actual manual drive
    public Command drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation) {
        return run(() -> deadbandMove(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotation.getAsDouble(), false));
    }

    // Yes! Remap?
    public Command robotCentricDrive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation) {
        return run(() -> deadbandMove(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotation.getAsDouble(), true))
                .withName("Robot Centric Drive");
    }

    public Command resetGyroCommand() {
        return runOnce(this::resetGyro).ignoringDisable(true);
    }

    public Command moveInDirection(double xSpeed, double ySpeed, double seconds) {
        return race(
                run(() -> {
                    move(xSpeed, ySpeed, 0, false);
                }),
                new FunctionalWaitCommand(seconds)).andThen(safeStateCmd());
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPipelineResult cameraResult) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = cameraResult.getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = getMap().visionEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) {
            return estStdDevs;
        }
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = kMultiTagStdDevs;
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        return estStdDevs;
    }

    @Override
    public void reset() {
        driveMap.gyro.reset();
    }

    @Override
    public void safeState() {
        move(0.0, 0.0, 0.0, false);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Use this for any background processing
        super.periodic();

        estimator.update(driveMap.gyro.getRotation2d(), getData().getModulePositions());
        // Correct pose estimate with vision measurements
        getMap().visionEstimator.update().ifPresent(est -> {
            var estPose = est.estimatedPose.toPose2d();
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs(estPose, getData().pipelineResult);

            estimator.addVisionMeasurement(estPose, est.timestampSeconds, estStdDevs);
        });

        Logger.recordOutput("estimatorPose", estimator.getEstimatedPosition());
        Logger.recordOutput("Angle", driveMap.gyro.getAngle());
        Logger.recordOutput("rotation", driveMap.gyro.getRotation2d());
        Logger.recordOutput("Actual Module States", getData().getModuleStates());
    }

    public void resetGyro() {
        driveMap.gyro.reset();
    }
}
