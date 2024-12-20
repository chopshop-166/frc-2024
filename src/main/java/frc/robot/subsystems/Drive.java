package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.logging.data.SwerveDriveData;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class Drive extends LoggedSubsystem<SwerveDriveData, SwerveDriveMap> {

    public final SwerveDriveKinematics kinematics;

    boolean isBlue = false;
    double maxDriveSpeedMetersPerSecond;
    double maxRotationRadiansPerSecond;
    double speedCoef = 1;
    double rotationCoef = 1;
    double rotationKp = 0.05;
    double rotationKs = 0.19;
    ProfiledPIDController rotationPID = new ProfiledPIDController(0.065, 0.0, 0.0, new Constraints(240, 270));
    double visionMaxError = 1;

    DoubleSupplier xSpeed;
    DoubleSupplier ySpeed;
    DoubleSupplier rotation;
    boolean isRobotCentric = false;
    boolean aimAtSpeaker = false;

    SwerveDrivePoseEstimator estimator;

    // Vision objects
    private PhotonCamera camera = null;
    private PhotonPoseEstimator photonEstimator = null;
    private double lastEstTimestamp = 0;

    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final Transform3d kRobotToCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(-6.9965), Units.inchesToMeters(-3.029),
                    Units.inchesToMeters(12.445)),
            new Rotation3d(0, Units.degreesToRadians(-16.875), Units.degreesToRadians(-6.5 + 180)));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public Drive(SwerveDriveMap map, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation) {

        super(new SwerveDriveData(), map);

        getMap().gyro.reset();
        kinematics = new SwerveDriveKinematics(map.frontLeft.getLocation(), map.frontRight.getLocation(),
                map.rearLeft.getLocation(), map.rearRight.getLocation());
        maxDriveSpeedMetersPerSecond = map.maxDriveSpeedMetersPerSecond;
        maxRotationRadiansPerSecond = map.maxRotationRadianPerSecond;

        estimator = new SwerveDrivePoseEstimator(kinematics, getMap().gyro.getRotation2d(),
                getData().getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.02, 0.02, 0.01),
                VecBuilder.fill(0.1, 0.1, 0.01));

        AutoBuilder.configureHolonomic(estimator::getEstimatedPosition, this::setPose,
                this::getSpeeds, this::move, // Method that will drive the robot given ROBOT
                // RELATIVE ChassisSpeeds
                map.pathFollower, () -> !isBlue, this);

        camera = new PhotonCamera("Arducam_OV9782_USB_Camera");

        photonEstimator = new PhotonPoseEstimator(
                kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        rotationPID.enableContinuousInput(-180, 180);

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotation = rotation;

    }

    public void setPose(Pose2d pose) {
        estimator.resetPosition(getMap().gyro.getRotation2d(),
                getData().getModulePositions(), pose);
    }

    public Command setPoseCommand(Supplier<Pose2d> pose) {
        return runOnce(() -> {
            setPose(pose.get());
        });
    }

    private void periodicMove(final double xSpeed, final double ySpeed,
            final double rotation, boolean isRobotCentric, boolean aimAtSpeaker) {

        var deadband = Modifier.scalingDeadband(0.05);
        double rotationInput = deadband.applyAsDouble(rotation);
        double xInput = deadband.applyAsDouble(xSpeed);
        double yInput = deadband.applyAsDouble(ySpeed);

        final double translateXSpeed = xInput
                * maxDriveSpeedMetersPerSecond * speedCoef;
        final double translateYSpeed = yInput
                * maxDriveSpeedMetersPerSecond * speedCoef;
        double rotationSpeed = rotationInput
                * maxRotationRadiansPerSecond * rotationCoef;
        if (aimAtSpeaker) {
            rotationSpeed = calculateRotateSpeedToTarget(this::getSpeakerTarget);
        }

        move(translateXSpeed, translateYSpeed, rotationSpeed, isRobotCentric);
    }

    private void move(final double xSpeed, final double ySpeed,
            final double rotation, boolean isRobotCentric) {

        // rotationOffset is temporary and startingRotation is set at the start
        ChassisSpeeds speeds;
        if (isRobotCentric) {
            speeds = new ChassisSpeeds(ySpeed, xSpeed, rotation);
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed,
                    rotation, estimator.getEstimatedPosition().getRotation());
        }

        move(speeds);

    }

    private void move(final ChassisSpeeds speeds) {

        // Now use this in our kinematics
        final SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxDriveSpeedMetersPerSecond);

        // All the states
        getData().setDesiredStates(moduleStates);
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getData().getModuleStates());
    }

    // Yes! Remap?
    public Command robotCentricDrive() {
        return startEnd(() -> {
            isRobotCentric = true;
        }, () -> {
            isRobotCentric = false;
        });
    }

    public Command aimAtSpeaker() {
        return startEnd(() -> {
            aimAtSpeaker = true;
        }, () -> {
            aimAtSpeaker = false;
        });
    }

    public Command moveInDirection(double xSpeed, double ySpeed, double seconds) {
        return run(() -> {
            move(xSpeed, ySpeed, 0, false);
        }).withTimeout(seconds).andThen(safeStateCmd());
    }

    public Command rotateToAngle(double targetAngle) {
        return run(() -> {
            rotateToAngleImpl(targetAngle);
        });
    }

    public Translation2d getSpeakerTarget() {
        Optional<Pose3d> pose;
        if (isBlue) {
            Logger.recordOutput("Alliance Speaker", "Blue");
            pose = kTagLayout.getTagPose(7);
        } else {
            Logger.recordOutput("Alliance Speaker", "Red/Other");
            pose = kTagLayout.getTagPose(4);
        }
        if (pose.isEmpty()) {
            return new Translation2d();
        }
        return pose.get().getTranslation().toTranslation2d();
    }

    public Translation2d getRobotToTarget(Translation2d target) {
        return target.minus(estimator.getEstimatedPosition().getTranslation());
    }

    public Command rotateToSpeaker() {
        return rotateTo(this::getSpeakerTarget);
    }

    public Command rotateTo(Supplier<Translation2d> target) {
        return run(() -> {
            double rotationSpeed = calculateRotateSpeedToTarget(target);
            move(0, 0, rotationSpeed, false);

        });
    }

    public double calculateRotateSpeedToTarget(Supplier<Translation2d> target) {
        var robotToTarget = getRobotToTarget(target.get());
        Logger.recordOutput("Target Pose", robotToTarget);
        return rotateToAngleImpl(robotToTarget.getAngle().getDegrees());
    }

    public Command rotateTo(Translation2d target) {
        return rotateTo(() -> target);
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets
     *         used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = photonEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult) {
            lastEstTimestamp = latestTimestamp;
            return visionEst;
        }
        return Optional.empty();
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = camera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = kTagLayout.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
            estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    @Override
    public void reset() {
        Rotation2d heading = isBlue ? new Rotation2d() : new Rotation2d(Math.PI);
        setPose(new Pose2d(estimator.getEstimatedPosition().getX(), estimator.getEstimatedPosition().getY(), heading));
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
        isBlue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
        estimator.update(getMap().gyro.getRotation2d(), getData().getModulePositions());

        // Correct pose estimate with vision measurements
        var visionEst = getEstimatedGlobalPose();
        // TODO: Remove this if vision actually works properly
        visionEst = Optional.empty();
        visionEst.ifPresent(est -> {
            var estPose = est.estimatedPose.toPose2d();
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs(estPose);
            Logger.recordOutput("Vision Pose", est.estimatedPose);
            estimator.addVisionMeasurement(est.estimatedPose.toPose2d(),
                    est.timestampSeconds, estStdDevs);
        });

        periodicMove(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotation.getAsDouble(), isRobotCentric, aimAtSpeaker);

        Logger.recordOutput("Estimator Pose", estimator.getEstimatedPosition());
        Logger.recordOutput("Pose Angle", estimator.getEstimatedPosition().getRotation());
        Logger.recordOutput("Robot Rotation Gyro", getMap().gyro.getRotation2d());
    }

    private double rotateToAngleImpl(double targetAngleDegrees) {
        targetAngleDegrees += 180;
        double estimatorAngle = estimator.getEstimatedPosition().getRotation().getDegrees();
        double rotationSpeed = rotationPID.calculate(estimatorAngle, targetAngleDegrees);
        rotationSpeed += Math.copySign(rotationKs, rotationSpeed);
        // need to ensure we move at a fast enough speed for gyro to keep up
        if (Math.abs(rotationSpeed) < 0.02 || Math.abs(rotationPID.getPositionError()) < 0.75) {
            rotationSpeed = 0;
        }
        Logger.recordOutput("Target Angle", targetAngleDegrees);
        Logger.recordOutput("Estimator Angle", estimatorAngle);
        Logger.recordOutput("Rotation Speed", rotationSpeed);
        Logger.recordOutput("Target Velocity", rotationPID.getSetpoint().velocity);
        Logger.recordOutput("Position Error", rotationPID.getPositionError());
        return rotationSpeed;
    }
}
