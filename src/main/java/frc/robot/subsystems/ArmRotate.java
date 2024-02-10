package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.motors.Modifier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.ArmRotateMap.Data;

public class ArmRotate extends LoggedSubsystem<Data, ArmRotateMap> {

    // Intake Position: 0 degrees <- base everything else off of it

    final ProfiledPIDController pid;
    // Set to zero until able to test
    final double RAISE_SPEED = .85;
    final double MANUAL_LOWER_SPEED_COEF = 0.1;
    final double SLOW_DOWN_COEF = 0.5;
    final double LOWER_SPEED = -0.1;
    final double NORMAL_TOLERANCE = 2;
    final double INTAKE_TOLERANCE = 4;

    private Constraints rotateConstraints = new Constraints(150, 200);
    ArmPresets level = ArmPresets.OFF;

    public enum ArmPresets {
        INTAKE(0.25),

        OFF(Double.NaN),

        SCORE_SPEAKER_PODIUM(33.4),

        SCORE_AMP(90),

        SCORE_SPEAKER_SUBWOOFER(11.5),

        SCORE_SPEAKER_CENTERLINE(28),

        STOW(75);

        // podium angle maybe: 33.4

        private double absoluteAngle;

        private ArmPresets(double absoluteAngle) {
            this.absoluteAngle = absoluteAngle;
        }

        /**
         * Get the angle of the arm
         * 
         * @return Arm angle in degrees
         */
        public double getAngle() {
            return absoluteAngle;
        }
    }

    public ArmRotate(ArmRotateMap armRotateMap) {
        super(new Data(), armRotateMap);
        pid = armRotateMap.pid;
    }

    private double getArmAngle() {
        return (getData().rotatingAbsAngleDegrees);
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
                getData().setPoint = limits(speed * speedCoef);
            } else if (level == ArmPresets.OFF) {
                getData().setPoint = 0;
            }

        });
    }

    public Command moveToZero() {
        return startEnd(
                () -> {
                    getData().setPoint = LOWER_SPEED;
                    level = ArmPresets.OFF;
                }, this::safeState).until(() -> {
                    return getArmAngle() < ArmPresets.INTAKE.getAngle();
                });
    }

    // Automatic rotation control (angle input)
    public Command moveTo(ArmPresets level) {
        // When executed the arm will move. The encoder will update until the desired
        // value is reached, then the command will end.
        PersistenceCheck setPointPersistenceCheck = new PersistenceCheck(30, pid::atGoal);
        return cmd("Move To Set Angle").onInitialize(() -> {
            this.level = level;
            if (level == ArmPresets.INTAKE) {
                pid.setTolerance(4);
            } else {
                pid.setTolerance(2);
            }
            pid.reset(getArmAngle(), getData().rotatingAngleVelocity);
        }).onExecute(() -> {
            Logger.recordOutput("Pid at goal", pid.atGoal());
        }).runsUntil(setPointPersistenceCheck);
    }

    private double limits(double speed) {

        speed = getMap().hardLimits.filterSpeed(getArmAngle(), speed);

        speed = getMap().softLimits.scaleSpeed(getArmAngle(), speed, SLOW_DOWN_COEF);

        return speed;
    }

    @Override
    public void reset() {
        // reset encoder?
    }

    @Override
    public void safeState() {
        getData().setPoint = 0;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (level != ArmPresets.OFF) {
            getData().setPoint = pid.calculate(getArmAngle(), new State(level.getAngle(), 0));
            getData().setPoint += getMap().armFeedforward.calculate(
                    Units.Degrees.of(pid.getSetpoint().position - 3.4).in(Units.Radians),
                    Units.DegreesPerSecond.of(pid.getSetpoint().velocity).in(Units.RadiansPerSecond));
        }
        Logger.recordOutput("armPreset", level);
    }

}