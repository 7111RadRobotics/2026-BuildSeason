package team7111.robot.utils.motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import team7111.robot.utils.motor.Motor.MechanismType;

public class MotorConfig {
    public final double gearRatio;
    public final boolean isInverted;
    public final boolean isBreakMode;
    public final PIDController pid;
    public final MechanismType mechanism;
    public ArmFeedforward armFF = null;
    public ElevatorFeedforward elevatorFF = null;
    public SimpleMotorFeedforward simpleFF = null;
    public TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    public SparkBaseConfig sparkConfig = new SparkMaxConfig();

    public MotorConfig(
        double gearRatio, boolean isInverted, boolean isBreakMode, 
        PIDController pid, MechanismType mechanism, double kS, double kV, double kA, double Kg
    ) {
        this.gearRatio = gearRatio;
        this.isInverted = isInverted;
        this.isBreakMode = isBreakMode;
        this.pid = pid;
        this.mechanism = mechanism;

        switch(mechanism){
            case flywheel:
                simpleFF = new SimpleMotorFeedforward(kS, kV, kA);
                break;
            case arm:
                armFF = new ArmFeedforward(kS, Kg, kV, kA);
                break;
            case elevator:
                elevatorFF = new ElevatorFeedforward(kS, Kg, kV, kA);
                break;
            default:
                break;
        }
    }

    public MotorConfig withTalonConfig(TalonFXConfiguration config){
        this.talonConfig = config;
        return this;
    }

    public MotorConfig withSparkConfig(SparkBaseConfig config){
        this.sparkConfig = config;
        return this;
    }
}
