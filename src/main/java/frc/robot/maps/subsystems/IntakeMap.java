package frc.robot.maps.subsystems;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LogName;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class IntakeMap implements LoggableMap<IntakeMap.Data> {
    public SmartMotorController motor;
    private BooleanSupplier sensor;
    public double grabSpeed;

    public IntakeMap() {
        this(new SmartMotorController(), () -> false, 0);

    }

    public IntakeMap(SmartMotorController motor, BooleanSupplier sensor, double grabSpeed) {
        this.motor = motor;
        this.sensor = sensor;
        this.grabSpeed = grabSpeed;

    }

    @Override
    public void updateData(Data data) {
        data.roller.updateData(motor);
        data.gamePieceDetected = sensor.getAsBoolean();
    }

    public static class Data extends DataWrapper {
        public MotorControllerData roller = new MotorControllerData();

        @LogName("Game Piece Detected")
        public boolean gamePieceDetected;
    }

}