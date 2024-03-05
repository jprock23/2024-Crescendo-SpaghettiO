package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Ports;

public class LED {

    private static LED instance;

    private Spark PWMLED;

    public LED() {

        PWMLED = new Spark(Ports.blinkin);
    }

    public void setBlue(){
        PWMLED.set(0.0);
    }

    public double get(){
        return PWMLED.get();
    }

    public void setDisco(){
        PWMLED.set(-1.0);
    }

    public void setGreen() {
        PWMLED.set(.87);
    }

    public void setRed() {
        PWMLED.set(-.35);
    }

    public static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }

        return instance;
    }
}