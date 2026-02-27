package team7111.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Zones extends SubsystemBase {
    
    private BooleanSupplier isRedAlliance;

    public Zones(BooleanSupplier isRedAlliance){
        this.isRedAlliance = isRedAlliance;
    }

    private boolean inAllianceZone(Pose2d pose, boolean isRedAlliance){
        if(isRedAlliance){
            return pose.getX() >= 11.9;
        }else{
            return pose.getX() <= 4.65;
        }
    }

    public boolean inAllianceZone(Pose2d pose){
        return inAllianceZone(pose, isRedAlliance.getAsBoolean());
    }

    public boolean inNeutralZone(Pose2d pose){
        return !inAllianceZone(pose, true) && !inAllianceZone(pose, false);
    }
}
