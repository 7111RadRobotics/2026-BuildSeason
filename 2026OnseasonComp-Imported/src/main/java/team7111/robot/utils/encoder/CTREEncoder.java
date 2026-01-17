package team7111.robot.utils.encoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import team7111.robot.Constants;

public class CTREEncoder implements GenericEncoder{

    private CANcoder encoder;

    public CTREEncoder(int id, CANcoderConfiguration config){
        encoder = new CANcoder(id, Constants.canbus);
        encoder.getConfigurator().apply(config);
    }

    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public void setPosition(Rotation2d rotation) {
        encoder.setPosition(rotation.getRotations());
    }
}
