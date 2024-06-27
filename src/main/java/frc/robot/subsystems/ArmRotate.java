package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.motors.Modifier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.ArmRotateMap.ArmPresets;
import frc.robot.maps.subsystems.ArmRotateMap.Data;

public class ArmRotate extends LoggedSubsystem<Data, ArmRotateMap> {

    final ProfiledPIDController pid;
    final double RAISE_SPEED = .85;
    final double MANUAL_LOWER_SPEED_COEF = 0.5;
    final double SLOW_DOWN_COEF = 0.5;
    final double LOWER_SPEED = -0.15;
    final double NORMAL_TOLERANCE = 2;
    final double INTAKE_TOLERANCE = 4;
    double holdAngle = 0;

    ArmPresets level = ArmPresets.OFF;

    public ArmRotate(ArmRotateMap armRotateMap) {
        super(new Data(), armRotateMap);
        pid = armRotateMap.pid;
    }

    private double getArmAngle() {
        return getData().rotatingAbsAngleDegrees;
    }

    // Manual rotation control
    public Command move(DoubleSupplier rotationSpeed) {
        Modifier deadband = Modifier.deadband(.1);

        return run(() -> {
            double speed = deadband.applyAsDouble(rotationSpeed.getAsDouble());
            double speedCoef = RAISE_SPEED;
            if (speed < 0) {
                speedCoef = MANUAL_LOWER_SPEED_COEF;
            }
            if (Math.abs(speed) > 0) {
                level = ArmPresets.OFF;
                getData().setSetpoint(limits(speed * speedCoef));
            } else if (level == ArmPresets.OFF) {
                getData().setSetpoint(0.0);
            }

        });
    }

    public Command moveToZero() {
        return startSafe(() -> {
            getData().setSetpoint(LOWER_SPEED);
            level = ArmPresets.OFF;
        }).until(() -> {
            return getArmAngle() < getMap().armPresets.getValue(ArmPresets.INTAKE);
        });
    }

    // Automatic rotation control (angle input)
    public Command moveTo(ArmPresets level) {
        // When executed the arm will move. The encoder will update until the desired
        // value is reached, then the command will end.
        PersistenceCheck setPointPersistenceCheck = new PersistenceCheck(30, pid::atGoal);
        return runOnce(() -> {
            this.level = level;
            if (level == ArmPresets.INTAKE) {
                pid.setTolerance(4);
            } else {
                pid.setTolerance(2);
            }
            pid.reset(getArmAngle(), getData().rotatingAngleVelocity);
        }).andThen(run(() -> {
            Logger.recordOutput("Pid at goal", pid.atGoal());
        }).until(setPointPersistenceCheck)).withName("Move To Set Angle");
    }

    public Command hold() {
        return runOnce(() -> {
            holdAngle = getArmAngle();
            this.level = ArmPresets.HOLD;
        });
    }

    private double limits(double speed) {
        double angle = getArmAngle();
        // If we're at the hard limits, don't allow it to go any further
        speed = getMap().hardLimits.filterSpeed(angle, speed);
        // If we're at the soft limits, take it slow
        speed = getMap().softLimits.scaleSpeed(angle, speed, SLOW_DOWN_COEF);

        return speed;
    }

    @Override
    public void reset() {
        // reset encoder?
    }

    @Override
    public void safeState() {
        getData().setSetpoint(0.0);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (level != ArmPresets.OFF) {
            double targetAngle = level == ArmPresets.HOLD ? holdAngle : getMap().armPresets.getValue(level);
            double setpoint = pid.calculate(getArmAngle(), new State(targetAngle, 0));
            setpoint += getMap().armFeedforward.calculate(
                    Units.Degrees.of(pid.getSetpoint().position).in(Units.Radians),
                    Units.DegreesPerSecond.of(pid.getSetpoint().velocity).in(Units.RadiansPerSecond));
            getData().setSetpoint(setpoint);
        }
        Logger.recordOutput("armPreset", level);
        Logger.recordOutput("desiredArmVelocity", pid.getSetpoint().velocity);
        Logger.recordOutput("desiredArmPosition", pid.getSetpoint().position);
    }

}