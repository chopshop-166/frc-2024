package frc.robot.subsystems;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.maps.subsystems.ShooterMap.Data;

public class Shooter extends LoggedSubsystem<Data, ShooterMap> {

    private final double MIN_VELOCITY_OFFSET = 50;
    private final double MAX_VELOCITY_OFFSET = 100;

    public enum Speeds {
        FULL_SPEED(3000, 5500, 4500),

        // 2500, 2500, 3000
        SUBWOOFER_SHOT(2700, 2700, 3000),

        SHUTTLE_SHOT(2300, 2300, 3000),

        // 3000, 3000, 3500
        PODIUM_SHOT(2500, 4500, 3500),

        AMP_SPEED(1125, 1125, 1125),

        OFF(0, 0, 0);

        private double leftSpeed;
        private double rightSpeed;
        private double bothSpeed;

        private Speeds(double leftSpeed, double rightSpeed, double bothSpeed) {
            this.leftSpeed = leftSpeed;
            this.rightSpeed = rightSpeed;
            this.bothSpeed = bothSpeed;
        }

        public double getLeftSpeed(boolean splitSpeeds) {
            return splitSpeeds ? leftSpeed : bothSpeed;
        }

        public double getRightSpeed(boolean splitSpeeds) {
            return splitSpeeds ? rightSpeed : bothSpeed;
        }
    }

    public Shooter(ShooterMap shooterMap) {
        super(new Data(), shooterMap);
    }

    public Command setSpeed(Speeds speed) {
        PersistenceCheck speedPersistenceCheck = new PersistenceCheck(5, () -> {
            return Math.abs(getData().leftWheels.velocity) > speed.getLeftSpeed(getMap().splitSpeeds)
                    - MIN_VELOCITY_OFFSET
                    && Math.abs(getData().rightWheels.velocity) > speed.getRightSpeed(getMap().splitSpeeds)
                            - MIN_VELOCITY_OFFSET
                    && Math.abs(getData().leftWheels.velocity) < (speed.getLeftSpeed(getMap().splitSpeeds)
                            + MAX_VELOCITY_OFFSET)
                    && Math.abs(getData().rightWheels.velocity) < (speed.getRightSpeed(getMap().splitSpeeds)
                            + MAX_VELOCITY_OFFSET);
        });
        return run(() -> {
            getData().leftWheels.setpoint = speed.getLeftSpeed(getMap().splitSpeeds);
            getData().rightWheels.setpoint = speed.getRightSpeed(getMap().splitSpeeds);
        }).until(() -> {
            return speedPersistenceCheck.getAsBoolean() || speed == Speeds.OFF;
        });

    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void safeState() {
        getData().leftWheels.setpoint = 0;
        getData().rightWheels.setpoint = 0;
    }

}
