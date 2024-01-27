package frc.robot.subsystems;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.maps.subsystems.ShooterMap.Data;

public class Shooter extends LoggedSubsystem<Data, ShooterMap> {

    public enum Speeds {
        FULL_SPEED(4500),

        HALF_SPEED(2250),

        SLOW_SPEED(1125),

        THREE_QUARTER_SPEED(3375),

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
        PersistenceCheck speedPersistenceCheck = new PersistenceCheck(5,
                () -> Math.abs(getData().motor.velocityInchesPerSec) > speed.getSpeed());
        return run(
                () -> {
                    getData().motor.setpoint = speed.getSpeed();

                }).until(speedPersistenceCheck);

    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void safeState() {
        getData().motor.setpoint = 0;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
