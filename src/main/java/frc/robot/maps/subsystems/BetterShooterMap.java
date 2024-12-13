package frc.robot.maps.subsystems;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class BetterShooterMap {
    public SmartMotorController frontMotor;
    public SmartMotorController rearMotor;
    public BooleanSupplier irSensor;
    

    public BetterShooterMap() {
        this(new SmartMotorController(), new SmartMotorController(), () -> false);
    }

    public BetterShooterMap(SmartMotorController frontMotor, SmartMotorController rearMotor, BooleanSupplier irSensor) {
        this.frontMotor = frontMotor;
        this.rearMotor = rearMotor;
        this.irSensor = irSensor;
    }

}
