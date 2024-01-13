package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.RobotUtils;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.SwerveDriveMap;
import frc.robot.maps.SwerveDriveMap.Data;

public class Drive extends LoggedSubsystem<Data, SwerveDriveMap> {

    Pose2d pose;
    public final SwerveDriveKinematics kinematics;

    boolean isBlue = true;
    double maxDriveSpeedMetersPerSecond;
    double maxRotationRadiansPerSecond;
    double speedCoef = 1;
    double rotationCoef = 1;
    HolonomicPathFollowerConfig HoloPath = new HolonomicPathFollowerConfig(
            // HolonomicPathFollowerConfig, this should likely live in your
            // Constants class
            new PIDConstants(0.2, 0.0, 0.05), // Translation PID constants (OFF_AXIS)
            new PIDConstants(0.001, 0.0, 0.0), // Rotation PID constants (OFF_AXIS)
            2.0, // Max module speed, in m/s
            0.381,
            // Drive base radius (OFF_AXIS) in meters. Distance from robot center to
            // furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
    );

    private Vision vision;

    public Drive(SwerveDriveMap map) {

        super(new Data(), map);

        getMap().gyro().reset();
        kinematics = new SwerveDriveKinematics(map.frontLeft().getLocation(), map.frontRight().getLocation(),
                map.rearLeft().getLocation(), map.rearRight().getLocation());
        maxDriveSpeedMetersPerSecond = map.maxDriveSpeedMetersPerSecond();
        maxRotationRadiansPerSecond = map.maxRotationRadianPerSecond();
        AutoBuilder.configureHolonomic(() -> pose, vision::setPose,
                this::getSpeeds, this::move, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                HoloPath, () -> isBlue, this);
    }

    private void deadbandMove(final double xSpeed, final double ySpeed,
            final double rotation, boolean isRobotCentric) {

        var deadband = RobotUtils.scalingDeadband(
                (DriverStation.isFMSAttached()) ? 0.05 : 0.15);
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
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed,
                    rotation,
                    Rotation2d.fromDegrees(getData().gyroYawPositionDegrees));
        }

        move(speeds);

    }

    private void move(final ChassisSpeeds speeds) {

        // Now use this in our kinematics
        final SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxDriveSpeedMetersPerSecond);
        //

        // Front left module state
        getData().frontLeft.desiredState = moduleStates[0];

        // Front right module state
        getData().frontRight.desiredState = moduleStates[1];

        // Back left module state
        getData().rearLeft.desiredState = moduleStates[2];

        // Back right module state
        getData().rearRight.desiredState = moduleStates[3];

        // All the states
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getData().frontLeft.getModuleStates(),
                getData().frontRight.getModuleStates(), getData().rearLeft.getModuleStates(),
                getData().rearRight.getModuleStates());
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
        return cmd().onInitialize(() -> {
            resetGyro();
        }).runsUntil(() -> {
            return true;
        }).runsWhenDisabled(true);
    }

    public Command setPose(Pose2d pose) {
        return runOnce(() -> vision.setPose(pose));
    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void safeState() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Use this for any background processing
        pose = vision.update(isBlue);
    }

    public void resetGyro() {
        getMap().gyro().reset();
    }
}
