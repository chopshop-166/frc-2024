package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PoseFilter {
    private LinearFilter xFilter;
    private LinearFilter yFilter;

    /**
     * Low Pass Filter for Pose2d data
     * 
     * @param cutoff the cutoff of the filters
     */
    public PoseFilter(double cutoff) {
        xFilter = LinearFilter.singlePoleIIR(1, cutoff);
        yFilter = LinearFilter.singlePoleIIR(1, cutoff);
    }

    /**
     * Calculate the filtered pose data
     * 
     * @param input the raw pose data
     * @return the filtered pose data
     */
    public Pose2d calculate(Pose2d input) {
        return new Pose2d(
                xFilter.calculate(input.getX()),
                yFilter.calculate(input.getY()),
                // Add back the 180 degrees
                Rotation2d.fromDegrees(input.getRotation().getDegrees())

        );
    }

}
