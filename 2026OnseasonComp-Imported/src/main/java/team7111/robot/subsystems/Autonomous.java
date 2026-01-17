package team7111.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.lib.pathfinding.Path;
import team7111.lib.pathfinding.WaypointConstraints;
import team7111.robot.utils.AutoAction;

public class Autonomous extends SubsystemBase {

    private WaypointConstraints translationConstraints = new WaypointConstraints(5, 0, 0.1);
    private WaypointConstraints rotationConstraints = new WaypointConstraints(720, 0, 5);
    
    public enum Autos {
        shootPreload,
    }

    public enum Paths {
        forward,
    }

    public Autonomous(){

    }

    public void periodic(){}

    public void simulationPeriodic(){}

    public List<AutoAction> getAutonomous(Autos autoName){
        ArrayList<AutoAction> auto = new ArrayList<>();
        // define each autonomous using a switch statement.
        // each auto is an array of "AutoAction's"
        return auto;
    }

    public Path getPath(Paths path){
        // define Path object for each Paths enum using a switch statement
        return null;
    }
}
