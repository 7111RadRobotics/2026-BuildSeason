package team7111.robot.subsystems;

import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.robot.subsystems.Autonomous.Autos;
import team7111.robot.subsystems.Swerve.SwerveState;
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
        //TODO: decide and create more states (and remove examples)
        example1,
        example2,
        autonomousEnter,
        autonomous,
        autonomousExit,
    }

    //Subsystem Variables
    private final Autonomous auto;
    private final Swerve swerve;
    private final Vision vision;
    private final Example example; // object for an example subsystem that controls a mechanism
    //TODO: decide and create other subsystems
    

    // Buttons of controllers can be assigned to booleans which are checked in various super states. 
    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);

    /** This represents the current superstate of the robot */
    private SuperState superState = SuperState.example1;

    private boolean inAuto = false;
    private int autoIndex = 0;
    private List<AutoAction> autoActions;

    /**
     * The constructor will take each subsystem as an argument and save them as objects in the class. 
     * @param subsystem represents a subsystem. 
     */
    public SuperStructure(Autonomous auto, Swerve swerve, Vision vision, Example example){
        this.auto = auto;
        this.swerve = swerve;
        this.vision = vision;
        this.example = example;

        this.swerve.setJoysickInputs(() -> driverController.getLeftY(), () -> driverController.getLeftX(), () -> driverController.getRightX());
        this.swerve.setDriveFieldRelative(true);
    }

    /**
     * This method runs every iteration (every 20ms). Actions like state management and stateless logic are run here.
     */
    public void periodic(){
        manageSuperState(superState);
    }

    /**
     * This method is run every iteration (20ms) only in simulation mode. Can be used to update simulated mechanisms or sensors.
     */
    public void simulationPeriodic(){}

    /**
     * This is called the state manager. It checks the value of the SuperState argument and calls the method associated with it.
     * <p>Each state will have its own case statement, returning its state method.
     * @param state the SuperState value
     * @return the condition of the state determined by the state method.
     */
    private boolean manageSuperState(SuperState state){
        switch(state){
            case example1:
                return example1();
            case example2:
                return example2();
            case autonomous:
                return autonomous();
            case autonomousEnter:
                return autonomousEnter();
            case autonomousExit:
                return autonomousExit();
            default:
                return defaultState(state);
        }
    }

    /**
     * Each of these methods, called "state methods" (state1-autonomousExit), represent a defined state.
     * When called by the state manager, it will set the states of different subsystems.
     * @return true if the state is complete. The condition could represent mechanisms at a setpoint, a beambreak trigger, a timer, etc.
     * Mainly used for autonomous routines.
     */
    private boolean example1(){
        return true;
    }

    private boolean example2(){
        return true;
    }

    /** 
     * The state of the robot when autonomous initializes.
     * <p>It will reset/initialize certain variables and set the auto to the selected list of {@link AutoAction}'s from the auto chooser in {@link Autonomous}
     * before switching to the autonomous state. It also calls the autonomous state so it does not have to wait an extra iteration
     * @return true since there is no logic to compute
     */
    private boolean autonomousEnter(){
        setSuperState(SuperState.autonomous);
        autoIndex = 0;
        autoActions = auto.getAutonomous(Autos.shootPreload);
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
        boolean isActionFinished = true;
        AutoAction autoAction;
        autoAction = autoActions.get(autoIndex);

        if (autoAction.isPath()){
            SwerveState swerveState = swerve.getSwerveState();
            if (!swerveState.equals(SwerveState.initializePath) && !swerveState.equals(SwerveState.runPath)){
                swerve.setPath(autoAction.getAsPath());
                swerve.setSwerveState(SwerveState.initializePath);
            }
            isActionFinished = autoAction.getAsPath().isPathFinished();
        } else if (autoAction.isState()) {
            isActionFinished = manageSuperState(autoAction.getAsState());
        }

        if (autoAction.hasAlternateCondition()){
            isActionFinished = autoAction.getAlternateCondition();
        }

        if (autoAction.hasAdditionalCondition()){
            isActionFinished = (isActionFinished && autoAction.getAdditionalCondition());
        }

        if (autoAction.hasOptionalCondition()){
            isActionFinished = (isActionFinished || autoAction.getOptionalCondition());
        }

        if (isActionFinished) {
            autoIndex += 1;
        }
        /* TODO: Write code that will run a list of AutoActions received from Autonomous.java
         * It needs to check whether it is a Path or SuperState and whether there is
         * an alternate condition (replaces the default), additional condition (AND), or optional condition (OR).
         * If the AutoAction is a state, it will call manageState(autoAction.getSuperState).
         * If the AutoAction is a Path, it will set the path in swerve to its path and set the swerve state to initializePath.
         * The autoIndex should be increased when the current AutoAction's condition is true (This includes taking into account alternate, additional, and optional conditions).
         * The default condition for a SuperState is the returned boolean from it's state method.
         * The default condition for a Path is its isFinished() method.
         */

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
        //TODO: Code for setting up teleop goes here.
        // this may include setting swerve to manual control, setting the superState to a different state, etc.
        return true;
    }

    /**
     * The method called when the state method of the managed SuperState is not called.
     * @param state
     * @return
     */
    private boolean defaultState(SuperState state){
        System.err.println("The SuperState \"" + state.name() + "\" does not have a state method or it was not called."
            + "\nPlease return the state method in a new case statement in manageState()");
        return true;
    }

    /**
     * This method is responsible for changing the robot's {@link SuperState}.
     * <p> This method MUST be used when changing the robot's SuperState to ensure it does not interfere with autonomous.
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
