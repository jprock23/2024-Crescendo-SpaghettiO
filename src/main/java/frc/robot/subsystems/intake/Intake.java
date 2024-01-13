package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Ports;
import frc.robot.subsystems.intake.IntakeStates.*;

public class Intake {
     
    private CANSparkMax roller;
    private CANSparkMax flipper;

    private IntakePID control;

    private IntakePosition intakePosition = IntakePosition.RETRACTED;
    public static Intake instance;
    
    public Intake() {
        roller = new CANSparkMax(Ports.roller, MotorType.kBrushless);
        roller.setInverted(false);
        roller.burnFlash();
        
        flipper = new CANSparkMax(Ports.flipper,MotorType.kBrushless);
        flipper.setInverted(false);
        flipper.burnFlash();

        control = IntakePID.getInstance(flipper);

    }

    public void periodic(){
        control.setIntakeSP(intakePosition.position);
    }

    public void setRollerPower(double power){
        roller.set(power);
    }

    public double getRollerCurrent() {
        return roller.getOutputCurrent();
    }

    public boolean hasReachedPose(double tolerance) {
                if (Math.abs(control.getFlipperPosition() - intakePosition.position) > tolerance) {
            return true;
        }
        return false;
    }

    public void setIntakeState(IntakePosition state) {
        this.intakePosition = state;
    }
    
    public static Intake getInstance() {
        if(instance == null)
            instance = new Intake();
        return instance;
    }
}