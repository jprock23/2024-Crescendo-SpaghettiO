package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Ports;

public class Launcher {

    private CANSparkMax flywheel1;
    private CANSparkMax flywheel2;
    private CANSparkMax ringPusher;
    private CANSparkMax launcherAngle1;
    private CANSparkMax launcherAngle2;
    public static Launcher instance;

    public Launcher() {
        flywheel1 = new CANSparkMax(Ports.flywheel1, MotorType.kBrushless);
        flywheel1.setInverted(false);
        flywheel1.burnFlash();

        flywheel2 = new CANSparkMax(Ports.flywheel2, MotorType.kBrushless);
        flywheel2.setInverted(true);
        flywheel2.burnFlash();

        ringPusher = new CANSparkMax(Ports.ringPusher, MotorType.kBrushless);
        ringPusher.setInverted(false);
        ringPusher.burnFlash();

        launcherAngle1 = new CANSparkMax(Ports.launcherAngle1, MotorType.kBrushless);
        launcherAngle1.setInverted(false);
        launcherAngle1.burnFlash();

        launcherAngle2 = new CANSparkMax(Ports.launcherAngle2, MotorType.kBrushless);
        launcherAngle2.setInverted(true);
        launcherAngle2.burnFlash();
    }

    public void setLauncherPower(double power) {
        flywheel1.set(power);
        flywheel2.set(power);
    }

    public static Launcher getInstance() {
        if(instance == null)
            instance = new Launcher();
        return instance;
    }
}
