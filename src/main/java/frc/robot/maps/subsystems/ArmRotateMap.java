package frc.robot.maps.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.MockEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ArmRotateMap implements LoggableMap<ArmRotateMap.Data> {

    public final SmartMotorController motor;
    public final ProfiledPIDController pid;
    public final ArmFeedforward armFeedforward;
    public final IEncoder encoder;
    public final double softMaxAngle;
    public final double softMinAngle;
    public double hardMaxAngle;
    public double hardMinAngle;
    private double previousRate = 0;

    public ArmRotateMap() {
        this(new SmartMotorController(), new ProfiledPIDController(0, 0, 0,
                new Constraints(0, 0)), new ArmFeedforward(0, 0, 0, 0), new MockEncoder(), 0,
                0, 0, 0);
    }

    public ArmRotateMap(SmartMotorController motor, ProfiledPIDController pid, ArmFeedforward armFeedforward,
            IEncoder encoder,
            double softMaxAngle, double softMinAngle,
            double hardMaxAngle, double hardMinAngle) {
        this.motor = motor;
        this.pid = pid;
        this.armFeedforward = armFeedforward;
        this.encoder = encoder;
        this.softMaxAngle = softMaxAngle;
        this.softMinAngle = softMinAngle;
        this.hardMaxAngle = hardMaxAngle;
        this.hardMinAngle = hardMinAngle;
    }

    @Override
    public void updateData(Data data) {
        motor.set(data.setPoint);
        data.degrees = motor.getEncoder().getDistance();
        data.velocityDegreesPerSecond = motor.getEncoder().getRate();
        data.currentAmps = motor.getCurrentAmps();
        data.tempCelcius = motor.getTemperatureC();
        data.acceleration = data.velocityDegreesPerSecond - previousRate;
        previousRate = data.velocityDegreesPerSecond;
        data.rotatingAbsAngleDegrees = encoder.getAbsolutePosition();
        data.jointEncPosition = encoder.getDistance();
        data.rotatingAngleVelocity = encoder.getRate();
        data.positionError = pid.getPositionError();
        data.velocityError = pid.getVelocityError();
    }

    public static class Data implements LoggableInputs {

        public double setPoint;
        public double degrees;
        public double velocityDegreesPerSecond;
        public double acceleration;
        public double[] currentAmps = new double[0];
        public double[] tempCelcius = new double[0];
        public double rotatingAbsAngleDegrees;
        public double rotatingAngleVelocity;
        public double positionError;
        public double velocityError;
        public double jointEncPosition;

        // Logs the values of the variables
        @Override
        public void toLog(LogTable table) {
            table.put("MotorSetpoint", setPoint);
            table.put("RotationDegrees", degrees);
            table.put("MotorVelocityDegreesPerSeconds", velocityDegreesPerSecond);
            table.put("MotorTempCelcius", tempCelcius);
            table.put("MotorCurrentAmps", currentAmps);
            table.put("MotorAcceleration", acceleration);
            table.put("rotatingAbsAngleDegrees", rotatingAbsAngleDegrees);
            table.put("rotatingAngleVelocity", rotatingAngleVelocity);
            table.put("PIDPositionError", positionError);
            table.put("PIDVelocityError", velocityError);
            table.put("jointEncPosition", jointEncPosition);
        }

        // Retrieves the values of the variables
        @Override
        public void fromLog(LogTable table) {
            this.setPoint = table.get("MotorSetpoint", setPoint);
            this.degrees = table.get("RotationDegrees", degrees);
            this.velocityDegreesPerSecond = table.get("MotorVelocityMetersPerSeconds", velocityDegreesPerSecond);
            this.currentAmps = table.get("MotorCurrentAmps", currentAmps);
            this.tempCelcius = table.get("MotorTempCelcius", tempCelcius);
            this.acceleration = table.get("MotorAcceleration", acceleration);
            this.rotatingAbsAngleDegrees = table.get("rotatingAbsAngleDegrees", rotatingAbsAngleDegrees);
            this.rotatingAngleVelocity = table.get("rotatingAngleVelocity", rotatingAngleVelocity);
            this.positionError = table.get("PIDPositionError", positionError);
            this.velocityError = table.get("PIDVelocityError", velocityError);
            this.jointEncPosition = table.get("jointEncPosition", jointEncPosition);
        }

    }
}