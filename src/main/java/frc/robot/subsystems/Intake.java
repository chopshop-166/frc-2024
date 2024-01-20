package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.IntakeMap;
import frc.robot.maps.subsystems.IntakeMap.Data;

public class Intake extends LoggedSubsystem<Data, IntakeMap> {

    private final double GRAB_SPEED = 0.75;

    public Intake(IntakeMap intakeMap) {
        super(new Data(), intakeMap);
    }

    public Command spinIn() {
        return runEnd(
                () -> {
                    getData().setPoint = GRAB_SPEED;
                }, () -> {
                    getData().setPoint = 0;
                });
    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void safeState() {
    }

    @Override
    public void periodic() {
        super.periodic();
    }

}