package frc.robot.maps.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class ShooterMap implements LoggableMap<ShooterMap.Data> {

    public final SmartMotorController motor;

    public ShooterMap() {
        this(new SmartMotorController());
    }

    public ShooterMap(SmartMotorController motor) {
        this.motor = motor;
    }

    @Override
    public void updateData(Data data) {
        motor.set(data.setPoint);
        data.velocityRotationsPerSec = motor.getEncoder().getRate();
        data.currentAmps = motor.getCurrentAmps();
        data.tempCelcius = motor.getTemperatureC();
    }

    public static class Data implements LoggableInputs {

        public double setPoint;
        public double velocityRotationsPerSec;
        public double[] currentAmps = new double[0];
        public double[] tempCelcius = new double[0];

        // Logs the values of the variables
        @Override
        public void toLog(LogTable table) {
            table.put("MotorSetpoint", setPoint);
            table.put("MotorVelocityRotationsPerSec", velocityRotationsPerSec);
            table.put("MotorCurrentAmps", currentAmps);
            table.put("MotorTempCelcius", tempCelcius);
        }

        // Retrieves the values of the variables
        @Override
        public void fromLog(LogTable table) {
            setPoint = table.get("MotorSetpoint", setPoint);
            velocityRotationsPerSec = table.get("MotorVelocityRotationsPerSec", velocityRotationsPerSec);
            currentAmps = table.get("MotorCurrentAmps", currentAmps);
            tempCelcius = table.get("MotorTempCelcius", tempCelcius);
        }

    }
}