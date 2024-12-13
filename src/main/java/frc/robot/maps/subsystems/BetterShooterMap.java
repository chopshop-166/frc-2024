package frc.robot.maps.subsystems;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class BetterShooterMap {
    public SmartMotorController FrontMotor;
    public SmartMotorController RearMotor;
    public BooleanSupplier sensor;

    public BetterShooterMap() {
        this(new SmartMotorController(), new SmartMotorController(), () -> false);
    }

    public BetterShooterMap(SmartMotorController FrontMotor, SmartMotorController RearMotor, BooleanSupplier sensor) {
        this.FrontMotor = FrontMotor;
        this.RearMotor = RearMotor;
        this.sensor = sensor;
    }
}