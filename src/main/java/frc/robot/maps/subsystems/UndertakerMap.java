package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class UndertakerMap implements LoggableMap<UndertakerMap.Data> {
    public SmartMotorController motor;

    public UndertakerMap() {
        this(new SmartMotorController());

    }

    public UndertakerMap(SmartMotorController motor) {
        this.motor = motor;

    }

    @Override
    public void updateData(Data data) {
        data.roller.updateData(motor);
    }

    public static class Data extends DataWrapper {
        public MotorControllerData roller = new MotorControllerData("Roller");

        public Data() {
            super("Undertaker");
        }
    }

}