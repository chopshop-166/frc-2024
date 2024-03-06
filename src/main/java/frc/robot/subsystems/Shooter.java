package frc.robot.subsystems;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.maps.subsystems.ShooterMap.Data;

public class Shooter extends LoggedSubsystem<Data, ShooterMap> {

    public enum Speeds {
        FULL_SPEED(4500),

        SUBWOOFER_SHOT(3000),

        PODIUM_SHOT(3500),

        HALF_SPEED(2250),

        AMP_SPEED(1125),

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
                () -> {
                    return Math.abs(getData().topRoller.velocity) > speed.getSpeed()
                            && Math.abs(getData().bottomRoller.velocity) > speed.getSpeed();
                });
        return run(
                () -> {
                    getData().topRoller.setpoint = speed.getSpeed();
                    getData().bottomRoller.setpoint = speed.getSpeed();

                }).until(speedPersistenceCheck);

    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void safeState() {
        getData().topRoller.setpoint = 0;
        getData().bottomRoller.setpoint = 0;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
