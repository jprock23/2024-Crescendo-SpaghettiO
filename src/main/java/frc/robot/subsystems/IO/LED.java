package frc.robot.subsystems.IO;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Ports;

public class LED {

    private static LED instance;

    private AddressableLED LEDs;

    private AddressableLEDBuffer buffer;

    private int length;

    private Spark PWMLED;

    public LED() {

        // PWMLED = new Spark(Ports.blinkin);

        LEDs = new AddressableLED(Ports.blinkin);

        buffer = new AddressableLEDBuffer(length);
        LEDs.setLength(length);

        LEDs.setData(buffer);
        LEDs.start();
    }

    public void start() {
        LEDs.start();
    }

    public void setBlue() {

        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 255);
        }


        // PWMLED.set(0.0);
    }

    // public double get(){
    // return PWMLED.get();
    // }

    public void setDisco() {
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