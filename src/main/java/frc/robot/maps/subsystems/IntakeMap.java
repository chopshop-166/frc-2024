package frc.robot.maps.subsystems;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.digital.CSDigitalInput;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeMap implements LoggableMap<IntakeMap.Data> {
    public SmartMotorController motor;
    private BooleanSupplier sensor;

    public IntakeMap() {
        this(new SmartMotorController(), () -> false);

    }

    public IntakeMap(SmartMotorController motor, BooleanSupplier sensor) {
        this.motor = motor;
        this.sensor = sensor;
    }

    @Override
    public void updateData(Data data) {
        motor.set(data.setPoint);
        data.gamePieceDetected = sensor.getAsBoolean();
    }

    public static class Data implements LoggableInputs {
        public boolean gamePieceDetected;
        public double setPoint;
        public double velocityRotationsPerSec;
        public double[] currentAmps = new double[0];
        public double[] tempCelcius = new double[0];

        // Logs the values of the variables
        @Override
        public void toLog(LogTable table) {
            table.put("Game Piece Detected", gamePieceDetected);
            table.put("MotorSetpoint", setPoint);
            table.put("MotorVelocityRotationsPerSec", velocityRotationsPerSec);
            table.put("MotorCurrentAmps", currentAmps);
            table.put("MotorTempCelcius", tempCelcius);
        }

        // Retrieves the values of the variables
        @Override
        public void fromLog(LogTable table) {
            gamePieceDetected = table.get("Game Piece Detected", gamePieceDetected);
            setPoint = table.get("MotorSetpoint", setPoint);
            velocityRotationsPerSec = table.get("MotorVelocityRotationsPerSec", velocityRotationsPerSec);
            currentAmps = table.get("MotorCurrentAmps", currentAmps);
            tempCelcius = table.get("MotorTempCelcius", tempCelcius);
        }

    }

}