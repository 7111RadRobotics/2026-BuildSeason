package team7111.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class is an example to how a subsystem looks and functions.
 * The name of the file and class should be what it controls
 */
public class Hopper extends SubsystemBase {
    
    /**
     * The enum that holds the values of the subsystem's states.
     * It's name should be the subsystem's followed by "State"
     */
    public enum HopperState {
        intake,
        shoot,
        idle,
        manual,
    }

    private HopperState currentState = HopperState.idle;

    public Hopper() {}

    public void periodic(){
        manageState();
    }

    public void simulationPeriodic(){

    }

    /**
     * This is the subsystem's state manager.
     * It calls the state method of the variable representing the subsystem's state.
     */
    private void manageState(){
        switch(currentState){
            case idle:
                idle();
                break;
            case intake:
                intake();
                break;
            case shoot:
                shoot();
                break;
            case manual:
                manual();
                break;
            default:
                break;
            
        }
    }

    // named differently to not overide a different method
    private void idleMode(){

    }

    private void intake(){

    }

    private void shoot(){

    }

    private void manual(){
        //System.out.println("Runs code for the manual state");
    }

    public void setState(HopperState state){
        this.currentState = state;
    }

    public HopperState getState(){
        return currentState;
    }
}
