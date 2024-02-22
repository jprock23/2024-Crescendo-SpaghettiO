package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;

public class Climber {

    private CANSparkMax climber1;
    private CANSparkMax climber2;
    public static Climber instance;

    private boolean[] connections = new boolean[4];

    private double pow = 0.25;

    public Climber() {
        climber1 = new CANSparkMax(Ports.climber1, MotorType.kBrushless);
        climber1.restoreFactoryDefaults();

        climber1.setSmartCurrentLimit(60);
        climber1.setIdleMode(IdleMode.kBrake);
        climber1.setInverted(false);
        climber1.burnFlash();

        climber2 = new CANSparkMax(Ports.climber2, MotorType.kBrushless);
        climber2.restoreFactoryDefaults();

        climber2.setSmartCurrentLimit(60);
        climber2.setInverted(true);
        climber2.setIdleMode(IdleMode.kBrake);
        climber2.burnFlash();
    }

    public void setClimbingPower() {
        climber1.set(pow);
        climber2.set(pow);
    }

    public void setReverseClimberPower() {
        climber1.set(-pow);
        climber2.set(-pow);
    }

    public void setClimberOff() {
        climber1.set(0.0);
        climber2.set(0.0);
    }

    public double getClimber1Current() {
        return climber1.getOutputCurrent();
    }

    public double getClimber2() {
        return climber2.getOutputCurrent();
    }

    public boolean[] climberConnections() {
        if (climber1.getBusVoltage() != 0) {
            connections[0] = true;
        } else {
            connections[0] = false;
        }

        if (climber1.getOutputCurrent() != 0) {
            connections[1] = true;
        } else {
            connections[1] = false;
        }

        if (climber2.getBusVoltage() != 0) {
            connections[2] = true;
        } else {
            connections[2] = false;
        }

        if (climber2.getOutputCurrent() != 0) {
            connections[3] = true;
        } else {
            connections[3] = false;
        }

        return connections;
    }

    public void printConnections() {
        SmartDashboard.putBoolean("Climber1 Voltage", connections[0]);
        SmartDashboard.putBoolean("Climber1 Current", connections[1]);
        SmartDashboard.putBoolean("Climber2 Voltage", connections[2]);
        SmartDashboard.putBoolean("Climber2 Current", connections[3]);
    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

}