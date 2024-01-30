package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.subsystems.intake.IntakeStates.*;

public class Intake {
     
    private CANSparkMax roller;
    private CANSparkMax flipper;

    // private IntakePID control;

    private IntakePosition intakePosition = IntakePosition.RETRACTED;
    public static Intake instance;

    private double power = .5;
    private double flip = 0.25;
    
    public Intake() {
        roller = new CANSparkMax(Ports.roller, MotorType.kBrushless);
        roller.restoreFactoryDefaults();

        roller.setSmartCurrentLimit(40);
        roller.setIdleMode(IdleMode.kCoast);
        roller.setInverted(false);
        roller.burnFlash();
        
        flipper = new CANSparkMax(Ports.flipper,MotorType.kBrushless);
        flipper.restoreFactoryDefaults();

        flipper.setSmartCurrentLimit(70);
        flipper.setIdleMode(IdleMode.kBrake);
        flipper.setInverted(false);
        flipper.burnFlash();

        // control = IntakePID.getInstance(flipper);

    }

    public void periodic(){
        // control.setIntakeSP(intakePosition.position);
        // SmartDashboard.putNumber("Intake Position", control.getFlipperPosition());
        SmartDashboard.putNumber("Intake Power", power);
    }

     

    public void reverse(){
        flipper.set(-power);
    }

    public void setRollerPower(){
        roller.set(power);
    }

    public void setFlipperPower(){
        flipper.set(flip);
    }

    public void setFlipperOff(){
        roller.set(0.0);
    }

    public void setRollerOff(){
        roller.set(0);
    }

    public void decreaseRoller(){
        power -= power;
    }

        public void increaseRoller(){
        power += power;
    }

    public double getRollerCurrent() {
        return roller.getOutputCurrent();
    }

    // public boolean hasReachedPose(double tolerance) {
    //             if (Math.abs(control.getFlipperPosition() - intakePosition.position) > tolerance) {
    //         return true;
    //     }
    //     return false;
    // }

    public void setIntakeState(IntakePosition state) {
        this.intakePosition = state;
    }
    
    public static Intake getInstance() {
        if(instance == null)
            instance = new Intake();
        return instance;
    }
}