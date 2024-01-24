package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
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
        data.motor.updateData(motor);
    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData("Roller");

        public Data() {
            super("Shooter");
        }
    }
}