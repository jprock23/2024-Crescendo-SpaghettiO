package frc.robot.subsystems.IO;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Ports;

public class LED {

    private static LED instance;

    private AddressableLED LEDs;

    private AddressableLEDBuffer buffer;

    private int length = 25;

    private Spark PWMLED;

    public LED() {

        // PWMLED = new Spark(Ports.blinkin);

        LEDs = new AddressableLED(Ports.blinkin);

        buffer = new AddressableLEDBuffer(length);
        LEDs.setLength(buffer.getLength());

        LEDs.setData(buffer);
        LEDs.start();
    }

    public void setBlue() {

        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 255 );
        }

        LEDs.setData(buffer);
        // PWMLED.set(.85);
        // Yellow = .67
        // Rainbow = -.89
        // Flashbang? = .35
        // Strobe = -.05
        // Shot = .33
        // Larson's Pattern = .19
    }

    public void setDisco() {
        PWMLED.set(-0.89);
    }

    public void setGreen() {
        // PWMLED.set(0.75);
        //Just Green - 0.75
                for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 255, 0);
        }
        LEDs.setData(buffer);
    }

    public void setProgress(int count){
        for(int i = 0; i < count; i++){
            buffer.setRGB(i, 0, 255, 0);
        }

        LEDs.setData(buffer);
    }

    public void setRed() {
        // PWMLED.set(0.61);
        //Just Red - 0.61
                for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 255, 0, 0);
        }

        LEDs.setData(buffer);
    }
    
    public void setOrange(){
        PWMLED.set(-.97);
    }

    public static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }

        return instance;
    }
}