package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.UndertakerMap;
import frc.robot.maps.subsystems.UndertakerMap.Data;

public class Undertaker extends LoggedSubsystem<Data, UndertakerMap> {

    private final double GRAB_SPEED = 0.75;
    private final double RELEASE_SPEED = -0.75;

    public Undertaker(UndertakerMap undertakerMap) {
        super(new Data(), undertakerMap);
    }

    public Command spinIn() {
        return runEnd(
                () -> {
                    getData().roller.setpoint = GRAB_SPEED;

                }, this::safeState);
    }

    public Command spinOut() {
        return runEnd(
                () -> {
                    getData().roller.setpoint = RELEASE_SPEED;
                }, this::safeState);
    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void safeState() {
        getData().roller.setpoint = 0;
    }

    @Override
    public void periodic() {
        super.periodic();
    }

}