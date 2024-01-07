package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Ports;

public class Intake {

    private CANSparkMax roller;
    private CANSparkMax flipper;
    public static Intake instance;
    
    public Intake() {
        roller = new CANSparkMax(Ports.roller, MotorType.kBrushless);
        roller.setInverted(false);
        roller.burnFlash();
        
        flipper = new CANSparkMax(Ports.flipper,MotorType.kBrushless);
        flipper.setInverted(false);
        flipper.burnFlash();
    }

    public void setIntakePowers(double rollerPower, double flipperPower) {
        roller.set(rollerPower);
        roller.set(flipperPower);
    }
    
    
    public static Intake getInstance() {
        if(instance == null)
            instance = new Intake();
        return instance;
    }
}