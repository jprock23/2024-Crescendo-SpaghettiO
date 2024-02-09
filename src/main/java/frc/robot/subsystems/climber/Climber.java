package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Climber extends SubsystemBase{

    private CANSparkMax climber1;
    private CANSparkMax climber2;
    public static Climber instance;

    private double pow = 0.25;

    public Climber() {
        climber1 = new CANSparkMax(Ports.climber1,MotorType.kBrushless);
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

    // public void setClimberPower(double up, double down) {

    //     if(up != 0){
    //         climber1.set(up);
    //         climber2.set(up);
    //     } else if(down != 0){
    //         climber1.set(down);
    //         climber2.set(down);
    //     } else {
    //         climber1.set(0);
    //         climber2.set(0);
    //     }
    // }

    public void setClimbingPower(){
        climber1.set(pow);
        climber2.set(pow);
    }

    public void reverseClimb(){
        climber1.set(-pow);
        climber2.set(-pow);
    }

    public void setClimberStop(){
        climber1.set(0.0);
        climber2.set(0.0);
    }

    public double getAmpClimber1(){
        return climber1.getOutputCurrent();

    }

    public double getAmpClimber2(){
        return climber2.getOutputCurrent();

    }

    public static Climber getInstance() {
        if(instance == null){
             instance = new Climber();
        }
        return instance;
    }


}