package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import frc.robot.maps.SwerveDriveMap.Data;

import org.littletonrobotics.junction.Logger;
import com.chopshop166.chopshoplib.RobotUtils;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.DrivePID;
import frc.robot.util.RotationPIDController;

public class Drive extends SmartSubsystemBase {

    private final SwerveDriveMap map;
    Data io;
    public final SwerveDriveKinematics kinematics;

    private Pose2d pose = new Pose2d();
    private final DrivePID drivePID;

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
            0.381,
            // Drive base radius (OFF_AXIS) in meters. Distance from robot center to
            // furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
    );

    private Vision vision;

    // Used for automatic alignment while driving
    private final RotationPIDController rotationPID;
    // Used for rotation correction while driving
    private final RotationPIDController correctionPID;

    private double latestAngle = 0;

    public Drive(SwerveDriveMap map) {
        this.map = map;
        this.map.gyro().reset();
        io = new Data();
        kinematics = new SwerveDriveKinematics(map.frontLeft().getLocation(), map.frontRight().getLocation(),
                map.rearLeft().getLocation(), map.rearRight().getLocation());
        maxDriveSpeedMetersPerSecond = map.maxDriveSpeedMetersPerSecond();
        maxRotationRadiansPerSecond = map.maxRotationRadianPerSecond();
        drivePID = map.pid();
        correctionPID = new RotationPIDController(0.01, 0.00001, 0.0);
        rotationPID = drivePID.copyRotationPidController();
        kinematics = new SwerveDriveKinematics(map.frontLeft().getLocation(), map.frontRight().getLocation(),
                map.rearLeft().getLocation(), map.rearRight().getLocation());
        // AutoBuilder.configureHolonomic(this::getPose, vision::setPose,
        // this::getSpeeds,this::move, // Method that will drive the robot given ROBOT
        // RELATIVE ChassisSpeeds
        // HoloPath, this);
    }

    private void deadbandMove(final double xSpeed, final double ySpeed,
            final double rotation, boolean isRobotCentric) {

        var deadband = RobotUtils.scalingDeadband(
                (DriverStation.isFMSAttached()) ? 0.05 : 0.15);
        double rotationInput = deadband.applyAsDouble(rotation);
        double xInput = deadband.applyAsDouble(xSpeed);
        double yInput = deadband.applyAsDouble(ySpeed);

        SmartDashboard.putNumber("Rotation Correction Error", latestAngle - map.gyro().getAngle());

        if (Math.abs(rotationInput) < 0.1
                && !(Math.abs(xInput) < 0.1 && Math.abs(yInput) < 0.1)) {
            rotationInput = correctionPID.calculate(map.gyro().getAngle(), latestAngle);
            rotationInput = (Math.abs(rotationInput) > 0.02) ? rotationInput : 0;
            Logger.recordOutput("pidOutput", rotationInput);
            Logger.recordOutput("pidError", correctionPID.getError());
        } else {
            latestAngle = map.gyro().getAngle();
        }
        Logger.recordOutput("latestAngle", latestAngle);
        Logger.recordOutput("robotAngle", map.gyro().getAngle());

        final double translateXSpeed = xInput
                * maxDriveSpeedMetersPerSecond * speedCoef;
        final double translateYSpeed = yInput
                * maxDriveSpeedMetersPerSecond * speedCoef;
        final double rotationSpeed = rotationInput
                * maxRotationRadiansPerSecond * rotationCoef;
        _move(translateXSpeed, translateYSpeed, rotationSpeed, isRobotCentric);
    }

    public void move(final double xSpeed, final double ySpeed,
            final double rotation) {
        _move(xSpeed, ySpeed, rotation, false);
    }

    private void _move(final double xSpeed, final double ySpeed,
            final double rotation, boolean isRobotCentric) {

        // rotationOffset is temporary and startingRotation is set at the start
        ChassisSpeeds speeds;
        if (isRobotCentric) {
            speeds = new ChassisSpeeds(ySpeed, xSpeed, rotation);
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed,
                    rotation,
                    Rotation2d.fromDegrees(io.gyroYawPositionDegrees));
        }

        move(speeds);

    }

    private void move(final ChassisSpeeds speeds) {

        // Now use this in our kinematics
        final SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxDriveSpeedMetersPerSecond);
        //

        // Front left module state
        io.frontLeft.desiredState = moduleStates[0];

        // Front right module state
        io.frontRight.desiredState = moduleStates[1];

        // Back left module state
        io.rearLeft.desiredState = moduleStates[2];

        // Back right module state
        io.rearRight.desiredState = moduleStates[3];

        // All the states
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(io.frontLeft.getModuleStates(),
                io.frontRight.getModuleStates(), io.rearLeft.getModuleStates(), io.rearRight.getModuleStates());
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
            resetTag();
        }).runsUntil(() -> {
            return true;
        }).runsWhenDisabled(true);
    }

    public void resetTag() {
        vision.sawTag = false;
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
        isBlue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
        // This method will be called once per scheduler run
        // Use this for any background processing
        Logger.processInputs(getName(), io);
        pose = vision.update(isBlue);
        Logger.recordOutput("robotPose", pose);
    }

    public void resetGyro() {
        map.gyro().reset();
        latestAngle = 0;
    }
}
