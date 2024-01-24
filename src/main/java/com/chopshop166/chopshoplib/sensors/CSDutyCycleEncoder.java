package com.chopshop166.chopshoplib.sensors;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class CSDutyCycleEncoder extends DutyCycleEncoder implements IAbsolutePosition {

    private double distancePerRotation = 1.0;
    private double positionOffset = 0;

    /**
     * Create the Duty Cycle Encoder
     *
     * @param channel The channel the sensor is connected to.
     */
    public CSDutyCycleEncoder(int channel) {
        super(channel);
    }

    /**
     * Create the Duty Cycle Encoder
     *
     * @param dutyCycle The DutyCycle object.
     */
    public CSDutyCycleEncoder(DutyCycle dutyCycle) {
        super(dutyCycle);
    }

    /**
     * Create the Duty Cycle Encoder
     *
     * @param source The Source object.
     */
    public CSDutyCycleEncoder(DigitalSource source) {
        super(source);
    }

    @Override
    public double getAbsolutePosition() {
        double distance = super.getDistance();
        // Before the sensor is initialized we will get the negative offset back.
        // Just return 0 if this happens.
        if (distance == -this.positionOffset) {
            return 0;
        }
        return distance % 360.0;
    }

    @Override
    public void setDistancePerRotation(double distancePerRotation) {
        // Call the parent setDistancePerRotation with the same value
        super.setDistancePerRotation(distancePerRotation);

        if (this.positionOffset > 1) {
            super.setPositionOffset(this.positionOffset / Math.abs(this.distancePerRotation));
        }
        // Store it locally so we can use it when getting the absolute position
        this.distancePerRotation = distancePerRotation;
    }

    @Override
    public double getDistancePerRotation() {
        return this.distancePerRotation;
    }

    @Override
    public void setPositionOffset(double offset) {
        // Store locally for use when getting absolute position
        positionOffset = offset;

        if (this.distancePerRotation != 1) {
            offset /= Math.abs(this.distancePerRotation);
        }
        super.setPositionOffset(offset);
    }

    @Override
    public double getPositionOffset() {
        return this.positionOffset;
    }
}
