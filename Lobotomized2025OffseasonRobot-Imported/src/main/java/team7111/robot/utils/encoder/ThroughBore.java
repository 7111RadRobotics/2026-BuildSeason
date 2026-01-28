package team7111.robot.utils.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class ThroughBore implements GenericEncoder {

    private Encoder m_encoder;

    private Rotation2d zeroOffset = new Rotation2d();

    public ThroughBore(int channelA, int channelB, double conversionFactor) {
        m_encoder = new Encoder(channelA, channelB);
        m_encoder.setDistancePerPulse(conversionFactor);
        
    }

    @Override
    public Rotation2d getPosition() {
       return Rotation2d.fromRotations(m_encoder.getDistance());
    }

    @Override
    public void setPosition(Rotation2d desired) {
        m_encoder.reset();
    }
}
