package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.pneumatics.RevDSolenoid;

public class Claw extends SmartSubsystemBase {

    RevDSolenoid piston;

    public Claw() {
       piston = new RevDSolenoid(0, 1)
    }

    public Command open() {

    }

    @Override
    public void safeState() {
        //TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented")
    }
}