package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.ArmRotateMap.Data;

public class ArmRotate extends LoggedSubsystem<Data, ArmRotateMap> {

    // Intake Position: 0 degrees <- base everything else off of it

    private boolean useAbsolute = true;
    final ProfiledPIDController pid;
    // Set to zero until able to test
    final double PIVOT_HEIGHT = 0.0;
    final double RAISE_SPEED = 0.0;
    final double MANUAL_LOWER_SPEED_COEF = 0.0;
    final double SLOW_DOWN_COEF = 0.0;
    final double DESCEND_SPEED = -0.0;
    final double NO_FALL = 0.0;
    private double armLength = 0.0;

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

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

    DoublePublisher anglePub = ntinst.getDoubleTopic("Arm/Angle").publish();

    public ArmRotate(ArmRotateMap armRotateMap) {
        super(new Data(), armRotateMap);
        pid = armRotateMap.pid;
    }

    private double getArmAngle() {
        return useAbsolute ? (getData().rotatingRelativeAngleDegrees) : getData().degrees;
    }

    // Manual rotation control
    public Command move(DoubleSupplier rotationSpeed) {
        return run(() -> {
            double speedCoef = RAISE_SPEED;
            if (rotationSpeed.getAsDouble() < 0) {
                speedCoef = MANUAL_LOWER_SPEED_COEF;
            }
            // For when we add limits: getData().setPoint =
            // limits(rotationSpeed.getAsDouble() *
            // speedCoef);
        });
    }

    // Automatic rotation control (angle input)
    public Command moveToAngle(double angle, Constraints rotateConstraints) {
        // When executed the arm will move. The encoder will update until the desired
        // value is reached, then the command will end.
        PersistenceCheck setPointPersistenceCheck = new PersistenceCheck(20, pid::atGoal);
        return cmd("Move To Set Angle").onInitialize(() -> {
            pid.reset(getArmAngle(), getData().rotatingAngleVelocity);
        }).onExecute(() -> {
            getData().setPoint = pid.calculate(getArmAngle(), new State(angle, 0), rotateConstraints) + NO_FALL;
            Logger.getInstance().recordOutput("getPositionErrors", pid.getPositionError());

        }).runsUntil(setPointPersistenceCheck).onEnd(() -> {
            getData().setPoint = NO_FALL;
        });
    }

    // Move to enum preset using angle-input command
    public Command moveTo(ArmPresets level) {
        return moveToAngle(level.getAngle(), new Constraints(150, 200));
    }

    /*
     * Need to input:
     * Limits
     * 
     */

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
        anglePub.set(getArmAngle());
    }

}