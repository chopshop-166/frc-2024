package frc.robot.maps.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.ValueRange;
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
    private double previousRate = 0;
    public ValueRange hardLimits;
    public ValueRange softLimits;

    public ArmRotateMap() {
        this(new SmartMotorController(), new ProfiledPIDController(0, 0, 0,
                new Constraints(0, 0)), new ArmFeedforward(0, 0, 0, 0),
                new MockEncoder(), new ValueRange(0, 0), new ValueRange(0, 0));
    }

    public ArmRotateMap(SmartMotorController motor, ProfiledPIDController pid, ArmFeedforward armFeedforward,
            IEncoder encoder, ValueRange hardLimits, ValueRange softLimits) {
        this.motor = motor;
        this.pid = pid;
        this.armFeedforward = armFeedforward;
        this.encoder = encoder;
        this.hardLimits = hardLimits;
        this.softLimits = softLimits;
    }

    @Override
    public void updateData(Data data) {
        motor.set(data.setPoint);
        data.currentAmps = motor.getCurrentAmps();
        data.tempCelcius = motor.getTemperatureC();
        data.rotatingAbsAngleDegrees = encoder.getAbsolutePosition();
        data.positionError = pid.getPositionError();
        data.rotatingAngleVelocity = encoder.getRate();
    }

    public static class Data implements LoggableInputs {

        public double setPoint;
        public double[] currentAmps = new double[0];
        public double[] tempCelcius = new double[0];
        public double rotatingAbsAngleDegrees;
        public double rotatingAngleVelocity;
        public double positionError;

        // Logs the values of the variables
        @Override
        public void toLog(LogTable table) {
            table.put("MotorSetpoint", setPoint);
            table.put("MotorTempCelcius", tempCelcius);
            table.put("MotorCurrentAmps", currentAmps);
            table.put("rotatingAbsAngleDegrees", rotatingAbsAngleDegrees);
            table.put("PIDPositionError", positionError);
            table.put("rotatingAngleVelocity", rotatingAngleVelocity);
        }

        // Retrieves the values of the variables
        @Override
        public void fromLog(LogTable table) {
            this.setPoint = table.get("MotorSetpoint", setPoint);
            this.currentAmps = table.get("MotorCurrentAmps", currentAmps);
            this.tempCelcius = table.get("MotorTempCelcius", tempCelcius);
            this.rotatingAbsAngleDegrees = table.get("rotatingAbsAngleDegrees", rotatingAbsAngleDegrees);
            this.positionError = table.get("PIDPositionError", positionError);
            this.rotatingAngleVelocity = table.get("rotatingAngleVelocity", rotatingAngleVelocity);
        }

    }
}