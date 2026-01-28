package team7111.robot.utils.singleaxisgyro;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GenericSwerveGyro {
    public Rotation2d getRotation();

    public void setRotation(Rotation2d rotation);

    public void setInverted(boolean isCCW);

    default public void update(){}
}
