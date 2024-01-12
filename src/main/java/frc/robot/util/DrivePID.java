package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivePID {
    
    private final ProfiledPIDController xPid;
    private final ProfiledPIDController yPid;
    private final RotationPIDController anglePid;
    private final Constraints constraints;

    public DrivePID() {
        this(0, 0, 0, 0, 0, 0, new Constraints(0, 0));
    }

    /**
     * Wrapper for three PIDController objects, for each cardinal direction and
     * rotation.
     * 
     * @param positionP   kP for x and y positions
     * @param positionI   kI for x and y positions
     * @param positionD   kD for x and y positions
     * @param angleP      kP for angle
     * @param angleI      kI for angle
     * @param angleD      kD for angle
     * @param constraints Constraints for the profiled PID controllers
     */
    public DrivePID(double positionP, double positionI, double positionD, double angleP, double angleI, double angleD,
            Constraints constraints) {
        xPid = new ProfiledPIDController(positionP, positionI, positionD, constraints);
        yPid = new ProfiledPIDController(positionP, positionI, positionD, constraints);
        anglePid = new RotationPIDController(angleP, angleI, angleD);
        this.constraints = constraints;
    }

    public void reset(Translation2d measurement) {
        xPid.reset(measurement.getX());
        yPid.reset(measurement.getY());
        anglePid.reset();
    }

    /**
     * Get the direction to move/rotate in based on a current pose and target pose
     * 
     * @param currentPose The current pose
     * @param targetPose  The target pose
     * @return The transformation needed to move in the direction of the target pose
     */
    public Transform2d calculate(Pose2d currentPose, Pose2d targetPose) {
        double x = xPid.calculate(currentPose.getX(), targetPose.getX());
        if (Math.abs(x) > 0.01) {
        x += Math.signum(x) * 0.15;
        }
        double y = yPid.calculate(currentPose.getY(), targetPose.getY());
        if (Math.abs(y) > 0.01) {
            y += Math.signum(y) * 0.15;
        }
        double angle_dps = anglePid.calculate(
                targetPose.getRotation().getDegrees(), currentPose.getRotation().getDegrees());
        if (Math.abs(angle_dps) > 0.1) {
            angle_dps += Math.signum(angle_dps) * 1;
        }
        return new Transform2d(
                new Translation2d(-y, -x), Rotation2d.fromDegrees(angle_dps));
        // X and Y need to be swapped here for some reason));
    }

    /**
     * Checks if the current pose matches the target pose
     * 
     * @param currentPose The current pose
     * @param targetPose  The target pose
     * @param deadband    The acceptable difference between the current and target
     *                    poses
     * @return if current pose and target pose match
     */
    public boolean isFinished(Pose2d currentPose, Pose2d targetPose, double deadband) {
        Transform2d error = targetPose.minus(currentPose);

        SmartDashboard.putNumberArray("pidError",
                new double[] { error.getX(), error.getY(), error.getRotation().getRadians() });

        return (Math.abs(error.getX()) < deadband) && (Math.abs(error.getY()) < deadband)
                && anglePid.atSetpoint(deadband, currentPose.getRotation().getDegrees(),
                        targetPose.getRotation().getDegrees());
    }

    /**
     * Get a PID Controller using the values from the translation PID Controller
     * 
     * @return the translation PID Controller
     */
    public ProfiledPIDController copyTranslationPidController() {
        return new ProfiledPIDController(xPid.getP(), xPid.getI(), xPid.getD(), constraints);
    }

    /**
     * Get a PID Controller using the values from the rotation PID Controller
     * 
     * @return the rotation PID Controller
     */
    public RotationPIDController copyRotationPidController() {
        return new RotationPIDController(anglePid.getP(), anglePid.getI(), anglePid.getD());
    }

}
