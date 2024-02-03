package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class ShooterMap implements LoggableMap<ShooterMap.Data> {

    public final SmartMotorController topRoller;
    public final SmartMotorController bottomRoller;

    public ShooterMap() {
        this(new SmartMotorController(), new SmartMotorController());
    }

    public ShooterMap(SmartMotorController topRoller, SmartMotorController bottomRoller) {
        this.topRoller = topRoller;
        this.bottomRoller = bottomRoller;
    }

    @Override
    public void updateData(Data data) {
        data.topRoller.updateData(topRoller);
        data.bottomRoller.updateData(bottomRoller);
    }

    public static class Data extends DataWrapper {
        public MotorControllerData topRoller = new MotorControllerData("Top Roller");
        public MotorControllerData bottomRoller = new MotorControllerData("Bottom Roller");

        public Data() {
            super("Shooter");
        }
    }
}