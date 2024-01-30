package frc.robot.subsystems.LEDS;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Lights {

    private AddressableLED lit;
    private AddressableLEDBuffer litBuffer;

    
    
    public void litSetter(){
        for (var i = 0; i < litBuffer.getLength(); i++) {
            
            litBuffer.setHSV(i, 0, 100, 100);
         }
         
    }

  
    


}
   
