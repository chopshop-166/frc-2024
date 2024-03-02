package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LogName;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.MockEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ArmRotateMap implements LoggableMap<ArmRotateMap.Data> {

    public enum ArmPresets {

        // Most values for comp robot are two degrees less than alpha bot

        INTAKE,

        OFF,

        SCORE_SPEAKER_PODIUM,

        SCORE_AMP,

        SCORE_SPEAKER_SUBWOOFER,

        // Probs not correct
        SCORE_SPEAKER_CENTERLINE,

        HOLD,

        STOW
    }

    public record ArmPresetValues(double intake, double amp, double scoreSpeakerCenterline,
            double scoreSpeakerPodium, double scoreSpeakerSubwoofer, double stow) {
        public ArmPresetValues() {
            this(0, 0, 0, 0, 0, 0);
        }

        public double getValue(ArmPresets preset) {
            switch (preset) {
                case INTAKE:
                    return intake;

                case SCORE_AMP:
                    return amp;

                case OFF:
                    return Double.NaN;

                case SCORE_SPEAKER_CENTERLINE:
                    return scoreSpeakerCenterline;

                case SCORE_SPEAKER_PODIUM:
                    return scoreSpeakerPodium;

                case SCORE_SPEAKER_SUBWOOFER:
                    return scoreSpeakerSubwoofer;

                case STOW:
                    return stow;
                default:
                    return 0;
            }

        }
    }

    public final SmartMotorController motor;
    public final ProfiledPIDController pid;
    public final ArmFeedforward armFeedforward;
    public final IEncoder encoder;
    private double previousRate = 0;
    public ValueRange hardLimits;
    public ValueRange softLimits;
    public ArmPresetValues armPresets;

    public ArmRotateMap() {
        this(new SmartMotorController(), new ProfiledPIDController(0, 0, 0,
                new Constraints(0, 0)), new ArmFeedforward(0, 0, 0, 0),
                new MockEncoder(), new ValueRange(0, 0), new ValueRange(0, 0), new ArmPresetValues());
    }

    public ArmRotateMap(SmartMotorController motor, ProfiledPIDController pid, ArmFeedforward armFeedforward,
            IEncoder encoder, ValueRange hardLimits, ValueRange softLimits, ArmPresetValues armPresets) {
        this.motor = motor;
        this.pid = pid;
        this.armFeedforward = armFeedforward;
        this.encoder = encoder;
        this.hardLimits = hardLimits;
        this.softLimits = softLimits;
        this.armPresets = armPresets;
    }

    @Override
    public void updateData(Data data) {
        data.motor.updateData(motor);
        data.acceleration = data.motor.velocityInchesPerSec - previousRate;
        previousRate = data.motor.velocityInchesPerSec;
        data.rotatingAbsAngleDegrees = encoder.getAbsolutePosition();
        data.positionError = pid.getPositionError();
        data.rotatingAngleVelocity = encoder.getRate();
    }

    public static class Data extends DataWrapper {

        public final MotorControllerData motor = new MotorControllerData();

        @LogName("Acceleration")
        public double acceleration;

        @LogName("Rotation Angle")
        public double rotatingAbsAngleDegrees;

        @LogName("Rotation Velocity")
        public double rotatingAngleVelocity;

        @LogName("PID Position Error")
        public double positionError;

        @LogName("PID Velocity Error")
        public double velocityError;

        public double getSetpoint() {
            return motor.setpoint;
        }

        public void setSetpoint(double setpoint) {
            motor.setpoint = setpoint;
        }
    }
}