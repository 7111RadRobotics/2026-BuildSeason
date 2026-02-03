package team7111.lib.pathfinding;

import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PosePlanning {

    private List<Pose2d> avoidPoses = new ArrayList<>();

    public void setAvoidPose(FieldElement[] fieldElements) {
        for (FieldElement element : fieldElements) {
            Pose2d[] poses;
            if (element.returnCorners() == null) {
                poses = element.returnCirclePose();
            } else {
                poses = element.returnCorners();
            }
            if (poses != null) {
                Collections.addAll(avoidPoses, poses);
            }
        }
    }

    public boolean isBlocked(Translation2d pos, double safetyRadius) {
        for (Pose2d obstacle : avoidPoses) {
            if (pos.getDistance(obstacle.getTranslation()) < safetyRadius) {
                return true;

            }
        }
        return false;
    }
}
