package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.race;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.RobotUtils;
import com.chopshop166.chopshoplib.commands.FunctionalWaitCommand;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.logging.data.SwerveDriveData;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class Drive extends LoggedSubsystem<SwerveDriveData, SwerveDriveMap> {

    public final SwerveDriveKinematics kinematics;

    boolean isBlue = false;
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
            0.3429,
            // Drive base radius (OFF_AXIS) in meters. Distance from robot center to
            // furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
    );

    SwerveDrivePoseEstimator estimator;

    public Drive(SwerveDriveMap map) {

        super(new SwerveDriveData(), map);

        getMap().gyro().reset();
        kinematics = new SwerveDriveKinematics(map.frontLeft().getLocation(), map.frontRight().getLocation(),
                map.rearLeft().getLocation(), map.rearRight().getLocation());
        maxDriveSpeedMetersPerSecond = map.maxDriveSpeedMetersPerSecond();
        maxRotationRadiansPerSecond = map.maxRotationRadianPerSecond();

        estimator = new SwerveDrivePoseEstimator(kinematics, getMap().gyro().getRotation2d(),
                getModulePositions(), new Pose2d(),
                VecBuilder.fill(0.02, 0.02, 0.01),
                VecBuilder.fill(0.1, 0.1, 0.01));

        AutoBuilder.configureHolonomic(estimator::getEstimatedPosition, this::setPose,
                this::getSpeeds, this::move, // Method that will drive the robot given ROBOT
                // RELATIVE ChassisSpeeds
                HoloPath, () -> isBlue, this);

    }

    public void setPose(Pose2d pose) {
        estimator.resetPosition(getMap().gyro().getRotation2d(),
                getModulePositions(), pose);
    }

    public Command setPoseCommand(Pose2d pose) {
        return runOnce(() -> {
            setPose(pose);
        });
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
                    getData().gyroYawPosition);
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

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] { getData().frontLeft.getModulePosition(),
                getData().frontRight.getModulePosition(), getData().rearLeft.getModulePosition(),
                getData().rearRight.getModulePosition() };
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getData().frontLeft.getModuleState(),
                getData().frontRight.getModuleState(), getData().rearLeft.getModuleState(),
                getData().rearRight.getModuleState());
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

    @Override
    public void reset() {
        // Nothing to reset here
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
        estimator.update(getMap().gyro().getRotation2d(), getModulePositions());
        Logger.recordOutput("pose", estimator.getEstimatedPosition());
        Logger.recordOutput("Angle", getMap().gyro().getAngle());
        Logger.recordOutput("rotation", getMap().gyro().getRotation2d());
    }

    public void resetGyro() {
        getMap().gyro().reset();
    }
}
