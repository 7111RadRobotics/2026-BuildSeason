package team7111.robot.utils.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class RelativeThroughBore implements GenericEncoder {

    private Encoder encoder;
    private double offset = 0;

    private Rotation2d zeroOffset = new Rotation2d();
    private double conversionFactor = 1;

    public RelativeThroughBore(int channelA, int channelB, double conversionFactor, double offset) {
        encoder = new Encoder(channelA, channelB);
        encoder.setDistancePerPulse(conversionFactor);
        this.conversionFactor = conversionFactor;
        this.offset = offset;
    }

    @Override
    public Rotation2d getPosition() {
       return Rotation2d.fromRotations((((double)encoder.get() / 2048.0) * 1/conversionFactor) * 360 + offset);
    }

    @Override
    public void setPosition(Rotation2d desired) {
        encoder.reset();
    }
}
