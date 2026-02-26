package team7111.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Zones extends SubsystemBase {
    
    private boolean isRedAlliance;

    public Zones(boolean isRedAlliance){
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
        return inAllianceZone(pose, isRedAlliance);
    }

    public boolean inNeutralZone(Pose2d pose){
        return !inAllianceZone(pose, true) && !inAllianceZone(pose, false);
    }
}
