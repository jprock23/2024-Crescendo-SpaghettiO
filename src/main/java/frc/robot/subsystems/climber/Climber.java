package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Ports;

public class Climber {

    private CANSparkMax climber1;
    private CANSparkMax climber2;
    public static Climber instance;

    public Climber() {
        climber1 = new CANSparkMax(Ports.climber1,MotorType.kBrushless);
        climber1.setInverted(false);
        climber1.burnFlash();

        climber2 = new CANSparkMax(Ports.climber2,MotorType.kBrushless);
        climber2.setInverted(true);
        climber2.burnFlash();
    }

    public void setClimberPower(double climber1Power, double climber2Power) {
        climber1.set(climber1Power);
        climber2.set(climber2Power);
    }

    public static Climber getInstance() {
        if(instance == null){
             instance = new Climber();
        }
        return instance;
    }


}