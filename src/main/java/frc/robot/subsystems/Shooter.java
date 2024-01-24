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

    public enum Speeds {
        FULL_SPEED(1.00),

        HALF_SPEED(.50),

        SLOW_SPEED(.25),

        THREE_QUARTER_SPEED(.75),

        OFF(0);

        private double speed;

        private Speeds(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return speed;
        }
    }

    public Shooter(ShooterMap shooterMap) {
        super(new Data(), shooterMap);
    }

    public Command setSpeed(Speeds speed) {
        return runOnce(
                () -> {
                    getData().setPoint = speed.getSpeed();

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
