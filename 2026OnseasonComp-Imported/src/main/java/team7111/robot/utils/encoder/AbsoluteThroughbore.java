package team7111.robot.utils.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AbsoluteThroughbore implements GenericEncoder {

    private DutyCycleEncoder encoder;

    public AbsoluteThroughbore(int channel){
        encoder = new DutyCycleEncoder(channel);
    }

    @Override
    public Rotation2d getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public void setPosition(Rotation2d rotation) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }
    
}
