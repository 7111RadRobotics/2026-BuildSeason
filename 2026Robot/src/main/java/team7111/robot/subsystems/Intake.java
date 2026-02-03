package team7111.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class is an example to how a subsystem looks and functions.
 * The name of the file and class should be what it controls
 */
public class Intake extends SubsystemBase {
    
    /**
     * The enum that holds the values of the subsystem's states.
     * It's name should be the subsystem's followed by "State"
     */
    public enum IntakeState {
        stow,
        deploy,
        intake,
        manual,
    }

    private IntakeState currentState = IntakeState.stow;

    public Intake() {}

    public void periodic(){
        manageState();
    }

    public void simulationPeriodic(){}

    // The below methods are examples of retrieveable boolean values.
    // These can be checked in SuperStructure to determine a SuperState
    // or change a state/value in another subsystem.
    public boolean isAtSetpoint(){
        boolean isAtSetpoint = true; // would be true if mechanisms were at/near their setpoints.
        return isAtSetpoint;
    }

    /**
     * This is the subsystem's state manager.
     * It calls the state method of the variable representing the subsystem's state.
     */
    private void manageState(){
        switch(currentState){
            case stow:
                stow();
                break;
            case deploy:
                deploy();
                break;
            case intake:
                intake();
                break;
            case manual:
                manual();
                break;
            default:
                break;
        }
    }

    private void stow(){
        //System.out.println("Runs code for the stow state");
    }

    private void deploy(){
        //System.out.println("Runs code for the deploy state");
    }

    private void intake(){}

    private void manual(){
        //System.out.println("Runs code for the manual state");
    }

    public void setState(IntakeState state){
        this.currentState = state;
    }

    public IntakeState getState(){
        return currentState;
    }
}
