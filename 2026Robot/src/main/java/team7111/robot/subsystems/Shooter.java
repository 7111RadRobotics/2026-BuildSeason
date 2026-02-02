package team7111.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.robot.utils.encoder.RelativeThroughBore;
import team7111.robot.utils.motor.ArmSimMotor;
import team7111.robot.utils.motor.CTREMotor;
import team7111.robot.utils.motor.FlywheelSimMotor;
import team7111.robot.utils.motor.Motor;
import team7111.robot.utils.motor.MotorConfig;
import team7111.robot.utils.motor.REVMotor;
import team7111.robot.utils.motor.TwoMotors;

/**
 * This class is an example to how a subsystem looks and functions.
 * The name of the file and class should be what it controls
 */
public class Shooter extends SubsystemBase {
    
    public enum ShooterState {
        idle,
        score,
        scoreAimbot,
        pass,
        stopped,
        manual,
    }

    private MotorConfig hoodConfig = new MotorConfig(
        0, isAtSetpoint(), isAtSetpoint(), null, null, 0, 0, 0, 0);
    private Motor shooterHood;
    private Motor shooterFlywheels;

    private ShooterState currentState = ShooterState.stopped;

    public Shooter() {
        shooterHood = RobotBase.isReal()
            ? new CTREMotor(1, null, null)
            : new ArmSimMotor(
                new RelativeThroughBore(0, 0, 0),
                new SingleJointedArmSim(
                    DCMotor.getKrakenX60(1), hoodConfig.gearRatio, 0.01, 0.2, 
                    Degrees.of(23).in(Radians), Degrees.of(53).in(Radians), false, 37, 0.1), 
                hoodConfig.pid, 
                hoodConfig.armFF);
        shooterFlywheels = RobotBase.isReal()
            ? new TwoMotors(
                new REVMotor(0, null, null), 
                new REVMotor(0, null, null))
            : new FlywheelSimMotor(null, null, null, null);
    }

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
            case idle:
                idleMode();
                break;
            case manual:
                manual();
                break;
            case pass:
                pass();
                break;
            case score:
                score();
                break;
            case scoreAimbot:
                scoreAimbot();
                break;
            case stopped:
                stopped();
                break;
            default:
                break;
        }
    }

    private void idleMode(){}

    private void pass(){}

    private void score(){}

    private void scoreAimbot(){}

    private void stopped(){}

    private void manual(){
        //System.out.println("Runs code for the manual state");
    }

    public void setState(ShooterState state){
        this.currentState = state;
    }

    public ShooterState getState(){
        return currentState;
    }
}
