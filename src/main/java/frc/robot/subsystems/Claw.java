package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.pneumatics.RevDSolenoid;

public class Claw extends SmartSubsystemBase {

    RevDSolenoid piston;

    public Claw(ClawMap map) {
       piston = map.piston;
    }

    public Command open() {
        return runOnce(() -> {
            piston.set(DoubleSolenoid.Value.kForward);
        });
    }

    public Command close() {
        return runOnce(() -> {
            piston.set(DoubleSolenoid.Value.kReverse);
        });
    }


    @Override
    public void safeState() {
        //TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented")
    }
}