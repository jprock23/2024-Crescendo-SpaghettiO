package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Ports;

public class LED {

    private static LED instance;

    private AddressableLED lights;
    private AddressableLEDBuffer buffer;

    private static final int length = 0;

    private Spark PWMLED;

    public LED() {

        PWMLED = new Spark(Ports.blinkin);

        // lights = new AddressableLED(Ports.blinkin);
        // buffer = new AddressableLEDBuffer(length);

        // lights.setLength(length);

        // lights.setData(buffer);
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

    // public void start() {
    // lights.start();
    // }

    // public void setData(){
    // lights.setData(buffer);
    // }

    // public void setRed() {
    // for (int i = 0; i < length; i++) {
    // buffer.setRGB(i, 255, 0, 0);
    // }
    // }

    // public void setGreen() {
    // for (int i = 0; i < length; i++) {
    // buffer.setRGB(i, 0, 255, 0);
    // }
    // }

    // public void setBlue() {
    // for (int i = 0; i < length; i++) {
    // buffer.setRGB(i, 255, 0, 255);
    // }
    // }

    public static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }

        return instance;
    }
}