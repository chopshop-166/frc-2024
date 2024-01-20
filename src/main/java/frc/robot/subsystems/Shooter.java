package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.maps.subsystems.ShooterMap.Data;

public class Shooter extends LoggedSubsystem<Data, ShooterMap> {

    private final double RELEASE_SPEED = 1.00;
    private final double AMP_SPEED = .50;
    private final double SLOW_SPEED = .25;
    private final double HALF_SPEED = .50;
    private final double THREE_QUARTER = .75;

    public Shooter(ShooterMap shooterMap) {
        super(new Data(), shooterMap);
    }

    public Command spinOut() {
        return runEnd(
                () -> {
                    getData().setPoint = RELEASE_SPEED;
                }, () -> {
                    getData().setPoint = 0;
                });
    }

    public Command Charge() {
        return runOnce(
                () -> {
                    getData().setPoint = RELEASE_SPEED;
                    waitSeconds(1); // Wait is temporary
                });
    }

    public Command ampSpin() {
        return startEnd(
                () -> {
                    getData().setPoint = AMP_SPEED;
                }, () -> {
                    getData().setPoint = 0;
                });
    }

    public Command fullPower() {
        return runOnce(
                () -> {
                    getData().setPoint = RELEASE_SPEED;
                });
    }

    public Command slowSpin() {
        return runOnce(() -> {
            getData().setPoint = THREE_QUARTER;
        });
    }

    public Command spinDown() {
        return runOnce(() -> {
            getData().setPoint = 0;
        });
    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void safeState() {
        getData().setPoint = 0;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
