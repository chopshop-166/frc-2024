package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.BetterShooterMap;

public class BetterShooter extends SmartSubsystemBase {

    private BetterShooterMap map;

    public BetterShooter(BetterShooterMap map) {
        this.map = map;
    }

    public Command shooter() {
        return this.runOnce(() -> {
            map.FrontMotor.set(1.0);
            map.RearMotor.set(1.0);
        }).until(() -> this.map.sensor.getAsBoolean() == false).andThen(safeStateCmd());
    }

    @Override
    public void safeState() {

    }
}
