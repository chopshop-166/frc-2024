package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.IntakeMap;
import frc.robot.maps.subsystems.IntakeMap.Data;

public class Intake extends LoggedSubsystem<Data, IntakeMap> {

    private final double GRAB_SPEED = 0.75;
    private final double RELEASE_SPEED = -0.75;
    private final double FEED_SPEED = 1.0;
    private final double FEED_DELAY = 0.2;

    public Intake(IntakeMap intakeMap) {
        super(new Data(), intakeMap);
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

    public Command intakeGamePiece() {
        return runEnd(
                () -> {
                    getData().roller.setpoint = GRAB_SPEED;
                }, this::safeState).until(() -> getData().gamePieceDetected);
    }

    public Command feedShooter() {
        return run(
                () -> {
                    getData().roller.setpoint = FEED_SPEED;
                }).until(() -> !getData().gamePieceDetected).andThen(waitSeconds(FEED_DELAY), runOnce(this::safeState));
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