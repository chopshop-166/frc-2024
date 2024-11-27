package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.pneumatics.IDSolenoid;
import com.chopshop166.chopshoplib.pneumatics.MockDSolenoid;

public class ClawMap {
    IDSolenoid piston; 


    public ClawMap(){
        this(new MockDSolenoid());
    }

    public ClawMap(IDSolenoid piston) {
        this.piston = piston;
    }
}
