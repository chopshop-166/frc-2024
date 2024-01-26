package com.chopshop166.chopshoplib.sensors;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class CSFusedEncoder implements IEncoder, Sendable {

    private final double ENCODER_THRESHOLD_DEGREES = 2;
    IEncoder relativeEncoder;
    IAbsolutePosition absolutePos;

    double relativeEncoderOffset;

    public CSFusedEncoder(IEncoder relativeEncoder, IAbsolutePosition absPosition) {
        this.relativeEncoder = relativeEncoder;
        this.absolutePos = absPosition;
        this.relativeEncoderOffset = this.absolutePos.getAbsolutePosition();
    }

    public double getRelativeEncoderOffset() {
        return this.relativeEncoderOffset;
    }

    private boolean distanceExceedsThreshold(double relativeDistance, double absoluteDistance) {
        return Math.abs(relativeDistance) - Math.abs(absoluteDistance) >= ENCODER_THRESHOLD_DEGREES;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Quadrature Encoder");
        builder.addDoubleProperty("Speed", this::getRate, null);
        builder.addDoubleProperty("Distance", this::getDistance, null);
        builder.addDoubleProperty("Absolute Position", this::getAbsolutePosition, null);
    }

    @Override
    public void reset() {
        // This sets the relative encoder to the current absolute position
        System.out.println("Reseting Fused Encoder");
        relativeEncoderOffset = this.absolutePos.getAbsolutePosition();
        this.relativeEncoder.reset();
    }

    @Override
    public double getDistance() {

        double distance = this.relativeEncoder.getDistance() + relativeEncoderOffset;
        // Keep trying to set the offset until we get a valid reading back
        if (relativeEncoderOffset == 0 || distanceExceedsThreshold(distance, this.absolutePos.getAbsolutePosition())) {
            reset();
            distance = this.relativeEncoder.getDistance() + relativeEncoderOffset;
        }
        return distance;
    }

    @Override
    public double getRate() {
        return this.relativeEncoder.getRate();
    }

    @Override
    public double getAbsolutePosition() {
        return this.absolutePos.getAbsolutePosition();
    }

}
