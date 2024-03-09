package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class ShooterMap implements LoggableMap<ShooterMap.Data> {

    public final SmartMotorController topRoller;
    public final SmartMotorController bottomRoller;
    public boolean splitSpeeds;

    public ShooterMap() {
        this(new SmartMotorController(), new SmartMotorController(), false);
    }

    public ShooterMap(SmartMotorController topRoller, SmartMotorController bottomRoller, boolean splitSpeeds) {
        this.topRoller = topRoller;
        this.bottomRoller = bottomRoller;
        this.splitSpeeds = splitSpeeds;
    }

    @Override
    public void updateData(Data data) {
        data.leftWheels.updateData(topRoller);
        data.rightWheels.updateData(bottomRoller);
    }

    public static class Data extends DataWrapper {
        public MotorControllerData leftWheels = new MotorControllerData(true);
        public MotorControllerData rightWheels = new MotorControllerData(true);
    }
}