package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.ArmRotateMap.Data;

public class ArmRotate extends LoggedSubsystem<Data, ArmRotateMap> {

    // Intake Position: 0 degrees <- base everything else off of it

    final ProfiledPIDController pid;
    // Set to zero until able to test
    final double RAISE_SPEED = 0.0;
    final double MANUAL_LOWER_SPEED_COEF = 0.0;
    final double SLOW_DOWN_COEF = 0.0;
    final double DESCEND_SPEED = -0.0;
    private Constraints rotateConstraints = new Constraints(150, 200);

    public enum ArmPresets {
        INTAKE(0),

        SCORE_AMP(0),

        SCORE_SPEAKER_SUBWOOFER(0);

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
        PersistenceCheck setPointPersistenceCheck = new PersistenceCheck(20, pid::atGoal);
        return cmd("Move To Set Angle").onInitialize(() -> {
            pid.reset(getArmAngle(), getData().rotatingAngleVelocity);
        }).onExecute(() -> {
            getData().setPoint = pid.calculate(getArmAngle(), new State(level.getAngle(), 0));
            getData().setPoint += getMap().armFeedforward.calculate(pid.getSetpoint().position,
                    pid.getSetpoint().velocity);

        }).runsUntil(setPointPersistenceCheck).onEnd(() -> {
            getData().setPoint = 0;
        });
    }

    /*
     * Need to input:
     * Limits
     * 
     */

    private double limits(double speed) {
        Logger.recordOutput("speed", speed);
        if (getArmAngle() < 0 && speed < 0) {
            return 0;
        }
        if ((getArmAngle() > getMap().hardMaxAngle && speed > 0) ||
                (getArmAngle() < getMap().hardMinAngle && speed < 0)) {
            return getData().setPoint = 0;
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