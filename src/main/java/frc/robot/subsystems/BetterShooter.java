package frc.robot.subsystems;


import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.BetterShooterMap;

public class BetterShooter extends SmartSubsystemBase {

    // Commands
    // shoot note
    // spin up to speed
    // stop

    private BetterShooterMap map;

    @Override
    public void safeState() {
        //
    }

    public BetterShooter(BetterShooterMap map) {
        this.map = map;
    }

    public Command spin() {
        return this.runOnce(() -> {
            map.frontMotor.set(1.0);
            map.rearMotor.set(-1.0);
        }).until(this.map.irSensor).andThen(safeStateCmd());
    }
    

    public Command shoot() {
        return this.runOnce(() -> {
            map.rearMotor.set(1.0);
        }).until(this.map.irSensor).andThen(safeStateCmd());
        
    }


}
