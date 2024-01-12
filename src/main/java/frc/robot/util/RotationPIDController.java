package frc.robot.util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class RotationPIDController {
    private PIDController pid;

    public RotationPIDController(double kP, double kI, double kD) {
        pid = new PIDController(kP, kI, kD);
    }

    private double getError(double measurementDegrees, double setpointDegrees) {
        double angleError = (setpointDegrees - measurementDegrees);

        if (Math.abs(angleError) > 180.0) {
            angleError = Math.signum(-angleError) * (360.0 - Math.abs(angleError));
        }
        return angleError;
    }

    public double calculate(double measurementDegrees, double setpointDegrees) {

        return pid.calculate(getError(measurementDegrees, setpointDegrees));
    }

    public boolean atSetpoint(double tolerance, double measurementDegrees, double setpointDegrees) {
        return Units.degreesToRadians(Math.abs(getError(measurementDegrees, setpointDegrees))) < tolerance;
    }

    public void reset() {
        pid.reset();
    }

    public double getError() {
        return pid.getPositionError();
    }

    public double getP() {
        return pid.getP();
    }

    public double getI() {
        return pid.getI();
    }

    public double getD() {
        return pid.getD();
    }
}
