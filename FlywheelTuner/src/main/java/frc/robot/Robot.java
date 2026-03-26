// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

    public boolean tuneKs = true;
    public boolean tuneKp = false;
    public boolean tuneKv = false;

    public boolean doesIncrement = true;

    public double kS = 0.0001;
    public double kV = 0.0;
    public double kP = 0.0;

    public double pidOutput = 0;

    public double increments[] = {0.001, 0.01, 0.1, 1, 10};
    public int index = 0;

    public SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV);
    public PIDController pid = new PIDController(kP, 0, 0);

    
    public TalonFX motor1 = new TalonFX(1);
    public TalonFX motor2 = new TalonFX(2);
    public Follower motorFollow = new Follower(1, MotorAlignmentValue.Opposed);

    public XboxController xboxController = new XboxController(0);

    public VelocityVoltage rpm = new VelocityVoltage(1500);

    

    

    public Robot() {
        
    }

    @Override
    public void robotPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {

        


    }

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {

        

       
        if (xboxController.getRightBumperButtonPressed()) {
             if (index >= 4) {
                index = 0;
            } else {
                index++;
            }
        } else if (xboxController.getLeftBumperButtonPressed()) {
            if (index <= 0) {
                index = 4;
            } else {
                index--;
            }
        }

        if (xboxController.getAButtonPressed()) {
           if (tuneKp) {
            tuneKs = true;
            tuneKv = false;
            tuneKp = false;
           } else if (tuneKs) {
            tuneKv = true;
            tuneKp = false;
            tuneKs = false;
           } else if (tuneKv) {
            tuneKp = true;
            tuneKs = false;
            tuneKv = false;
           }
        }



        if (xboxController.getPOV() == 0 && doesIncrement) {

            if (tuneKp) {
                kP += increments[index];
            } else if (tuneKs) {
                kS += increments[index];
            } else if (tuneKv) {
                kV += increments[index];
            }
        
            doesIncrement = false;
        } else if(xboxController.getPOV() == 180 && doesIncrement) {

            if (tuneKp) {
                kP -= increments[index];
            } else if (tuneKs) {
                kS -= increments[index];
            } else if (tuneKv) {
                kV -= increments[index];
            }

            doesIncrement = false;

        } else if (xboxController.getPOV() == -1) {
            doesIncrement = true;
        }
        
        

        pid.setP(kP);
        ff.setKs(kS);
        ff.setKv(kV);

        

        pidOutput = pid.calculate(motor1.getVelocity().getValueAsDouble() / 60, 1500) + ff.calculate(1500);

        motor1.setControl(new VelocityVoltage(pidOutput));
        motor2.setControl(motorFollow);

        SmartDashboard.putNumber("indexNum", index);
        SmartDashboard.putNumber("kP", kP);
        SmartDashboard.putNumber("kV", kV);
        SmartDashboard.putNumber("kS", kS);
        SmartDashboard.putNumber("increment", increments[index]);
        SmartDashboard.putBoolean("changekS", tuneKs);
        SmartDashboard.putBoolean("changekV", tuneKv);
        SmartDashboard.putBoolean("changekP", tuneKp);
        SmartDashboard.putBoolean("willIncrement", doesIncrement);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
