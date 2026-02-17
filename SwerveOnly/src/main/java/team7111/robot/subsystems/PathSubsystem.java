package team7111.robot.subsystems;

import java.lang.reflect.Array;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.lib.pathfinding.Path;
import team7111.lib.pathfinding.Waypoint;
import team7111.lib.pathfinding.WaypointConstraints;

public class PathSubsystem extends SubsystemBase{
    
    private Path currentPath;
    private SuperStructure superStructure;
    private ArrayList<Object> selectedAuto = new ArrayList<>();

    /**
     * Defaults current path to be null.
     */
    public PathSubsystem(SuperStructure superStructure) 
    {
            this.superStructure = superStructure;
    }

    public enum Paths {
        home,
        straight,
    };



    public enum Autos {
        forwardAuto,

    }

    public void periodic() {

    }

    public Path getPath(Paths pathName) {
        Waypoint[] waypoints;

        switch (pathName) {
            case home:
                waypoints = new Waypoint[]{};
                break;
            case straight:
                waypoints = new Waypoint[] {
                    new Waypoint(
                        new Pose2d(0, 1, new Rotation2d()), 
                        new WaypointConstraints(5, 0, 0.1), 
                        new WaypointConstraints(360, 0, 5)
                    ),
                };
                break;
            default:
                waypoints = new Waypoint[]{};
        }

        return new Path(waypoints);
    }

}
