package team7111.robot.utils.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AbsoluteThroughbore implements GenericEncoder {

    private DutyCycleEncoder encoder;
    private double offset;

    public AbsoluteThroughbore(int channel){
        encoder = new DutyCycleEncoder(channel);
    }

    public AbsoluteThroughbore(int channel, double offset) {
        encoder = new DutyCycleEncoder(channel);
        this.offset = offset;
    }

    @Override
    public Rotation2d getPosition() {
        // TODO Auto-generated method stub
        // Make sure to include the offset in the position
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public void setPosition(Rotation2d rotation) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }
    
}
