package frc.robot.subsystems;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.maps.subsystems.ShooterMap.Data;

public class Shooter extends LoggedSubsystem<Data, ShooterMap> {

    public enum Speeds {
        FULL_SPEED(3000, 5500, 4500),

        SUBWOOFER_SHOT(3000, 3000, 3000),

        PODIUM_SHOT(3500, 3500, 3500),

        HALF_SPEED(2250, 2250, 2250),

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
        PersistenceCheck speedPersistenceCheck = new PersistenceCheck(5,
                () -> {
                    return Math.abs(getData().leftWheels.velocity) > speed.getLeftSpeed(getMap().splitSpeeds)
                            && Math.abs(getData().rightWheels.velocity) > speed.getRightSpeed(getMap().splitSpeeds);
                });
        return run(
                () -> {
                    getData().leftWheels.setpoint = speed.getLeftSpeed(getMap().splitSpeeds);
                    getData().rightWheels.setpoint = speed.getRightSpeed(getMap().splitSpeeds);

                }).until(speedPersistenceCheck);

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

    @Override
    public void periodic() {
        super.periodic();
    }
}
