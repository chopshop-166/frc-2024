package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;

public class Drive extends SmartSubsystemBase {

    private final SwerveDriveMap map;

    public Drive(SwerveDriveMap map) {
        this.map = map;
    }

    @Override
    public void safeState() {
    }

}
