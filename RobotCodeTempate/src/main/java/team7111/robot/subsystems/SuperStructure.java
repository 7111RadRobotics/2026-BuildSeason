package team7111.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import team7111.robot.utils.AutoAction;
import team7111.lib.pathfinding.*;

/**
 * This class handles the overall state of the robot.
 * It does this by defining various "SuperState" values and calling the methods assosiated with each one.
 * Each method uses logic to determine subsystem states, when to switch SuperStates, and what SuperState to go to next.
 * Logic used can be button inputs, subsystem states, or other subsystem conditions.
 */
public class SuperStructure extends SubsystemBase {
    /**
     * This enumeration contains the values or "states" which determine the subsystems.
     * The top 2 state names are placeholders. 
     */
    public enum SuperState {
        state1,
        state2,
        autonomousEnter,
        autonomous,
        autonomousExit,
    }

    /** This represents a subsystem*/
    private final Example example;

    /** Buttons of controllers can be assigned to booleans which are checked in various super states. */
    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);

    private SuperState superState = SuperState.state1;

    private boolean inAuto = false;
    private int autoIndex = 0;

    /**
     * The constructor will take each subsystem as an argument and save them as objects in the class. 
     * @param subsystem represents a subsystem. 
     */
    public SuperStructure(Example example){
        this.example = example;
    }

    /**
     * This method runs every iteration (every 20ms). Actions like state management and stateless logic are run here.
     */
    public void periodic(){

    }

    /**
     * This method is run every iteration (20ms) only in simulation mode. Can be used to update simulated mechanisms or sensors.
     */
    public void simulationPeriodic(){

    }

    /**
     * This is called the state manager. It checks the value of the SuperState argument and calls the method associated with it.
     * @param state the SuperState value
     * @return the condition of the state determined by the state method.
     */
    private boolean manageSuperState(SuperState state){
        switch(state){
            case state1:
                return state1();
            case state2:
                return state2();
            case autonomous:
                return autonomous();
            case autonomousEnter:
                return autonomousEnter();
            case autonomousExit:
                return autonomousExit();
            default:
                return true;
        }
    }

    /**
     * Each of these methods, called "state methods" (state1-autonomousExit), represent a defined state.
     * When called by the state manager, it will set the states of different subsystems.
     * @return true if the state is complete. The condition could represent mechanisms at a setpoint, a beambreak trigger, a timer, etc.
     * Mainly used for autonomous routines.
     */
    private boolean state1(){
        return true;
    }

    private boolean state2(){
        return true;
    }

    /** 
     * The state of the robot when autonomous initializes.
     * <p>It will reset/initialize certain variables and set the auto to the selected list of {@link AutoAction}'s from the auto chooser in {@link Autonomous}
     * before switching to the autonomous state. It also calls the autonomous state so it does not have to wait an extra iteration
     * @return true since there is no logic to compute
     */
    private boolean autonomousEnter(){
        autoIndex = 0;
        superState = SuperState.autonomous;
        autonomous();
        return true;
    }

    /**
     * The state of the robot during the full autonomous period.
     * This will run each {@link AutoAction} from the list received from {@link Autonomous} in order.
     * <p>If there is an alternative condition in the AutoAction, this method should increment the autoIndex variable after the condition is true.
     * <p>Otherwise, if the AutoAction is a {@link Path}, this method should increment the variable when the path is finished;
     * if it's a {@link SuperState}, this method should increment the variable when the SuperState's state method returns true 
     * @return true if autoIndex >= the size of the AutoAction List
     */
    private boolean autonomous(){
        inAuto = true;

        return true;
    }

    /**
     * The state of the robot at the end of the autonomous period.
     * <p>This will set the robot up for Teleop control by setting inAuto to false, 
     * setting the swerve state to manual, and changing the superState (using setSuperState()) to a different state
     * @return true 
     */
    private boolean autonomousExit(){
        inAuto = false;
        return true;
    }

    /**
     * This method is responsible for changing the robot's {@link SuperState}.
     * <p> This method MUST be used when changing the robot's SuperState as it should not change when in autonomous mode.
     * @param state the SuperState to set the robot if not in Autonomous mode
     */
    public void setSuperState(SuperState state){
        if(inAuto)
            superState = state;
    }

    /**
     * Gets the current {@link SuperState} of the robot.
     * @return the value of the superState object
     */
    public SuperState getSuperState(){
        return superState;
    }
}
