package frc.robot.maps.subsystems.patterns;

import com.chopshop166.chopshoplib.leds.AnimatedPattern;
import com.chopshop166.chopshoplib.leds.SegmentBuffer;
import edu.wpi.first.wpilibj.util.Color;

/** Display a spinning dot. */
public class ReverseSpinPattern extends AnimatedPattern {
    /** The color of the dot. */
    private final Color color;
    /** The position of the indicator. */
    private int ledPosition;

    /** Constructor. */
    public ReverseSpinPattern() {
        this(Color.kGreen);
    }

    /**
     * Constructor.
     * 
     * @param color The color of the dot.
     */
    public ReverseSpinPattern(final Color color) {
        super(0.05);
        this.color = color;
    }

    @Override
    public void initialize(final SegmentBuffer buffer) {
        super.initialize(buffer);
        this.ledPosition = buffer.getLength() - 1;
    }

    @Override
    public void animate(final SegmentBuffer buffer) {
        this.ledPosition--;
        if (this.ledPosition < 0) {
            this.ledPosition = buffer.getLength() - 1;
        }
        buffer.setAll(Color.kBlack);
        buffer.set(this.ledPosition, this.color);
    }

    @Override
    public String toString() {
        return String.format("SpinPattern(%s)", this.color.toString());
    }
}
