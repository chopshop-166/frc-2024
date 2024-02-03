package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.ArmRotateMap.Data;

public class ArmRotate extends LoggedSubsystem<Data, ArmRotateMap> {

    // Intake Position: 0 degrees <- base everything else off of it

    final ProfiledPIDController pid;
    // Set to zero until able to test
    final double RAISE_SPEED = .25;
    final double MANUAL_LOWER_SPEED_COEF = 0.1;
    final double SLOW_DOWN_COEF = 0.5;
    private Constraints rotateConstraints = new Constraints(150, 200);

    public enum ArmPresets {
        INTAKE(0),

        SCORE_AMP(90),

        SCORE_SPEAKER_SUBWOOFER(10.5);

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
        return run(() -> {
            double speed = rotationSpeed.getAsDouble();
            double speedCoef = RAISE_SPEED;
            if (speed < 0) {
                speedCoef = MANUAL_LOWER_SPEED_COEF;
            }
            getData().setPoint = limits(speed * speedCoef);
        });
    }

    // Automatic rotation control (angle input)
    public Command moveTo(ArmPresets level) {
        // When executed the arm will move. The encoder will update until the desired
        // value is reached, then the command will end.
        PersistenceCheck setPointPersistenceCheck = new PersistenceCheck(30, pid::atGoal);
        return cmd("Move To Set Angle").onInitialize(() -> {
            Logger.recordOutput("State", "starting");
            pid.reset(getArmAngle(), getData().rotatingAngleVelocity);
        }).onExecute(() -> {
            getData().setPoint = pid.calculate(getArmAngle(), new State(level.getAngle(), 0));
            Logger.recordOutput("Pid setpoint", getData().setPoint);
            getData().setPoint += getMap().armFeedforward.calculate(
                    Units.DegreesPerSecond.of(pid.getSetpoint().position).in(Units.RadiansPerSecond),
                    Units.DegreesPerSecond.of(pid.getSetpoint().velocity).in(Units.RadiansPerSecond));
            Logger.recordOutput("RotationVelocity", pid.getSetpoint().velocity);
            Logger.recordOutput("Pid at goal", pid.atGoal());
            Logger.recordOutput("State", "running");
        }).runsUntil(setPointPersistenceCheck).onEnd(() -> {
            getData().setPoint = 0;
            Logger.recordOutput("State", "Finished");
        });
    }

    private double limits(double speed) {
        Logger.recordOutput("speed", speed);
        if (getArmAngle() < 0 && speed < 0) {
            return 0;
        }
        if ((getArmAngle() > getMap().hardMaxAngle && speed > 0) ||
                (getArmAngle() < getMap().hardMinAngle && speed < 0)) {
            return 0;
        }
        if ((getArmAngle() > getMap().softMaxAngle && speed > 0) ||
                (getArmAngle() < getMap().softMinAngle && speed < 0)) {
            return (speed * SLOW_DOWN_COEF);
        }

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
    }

}