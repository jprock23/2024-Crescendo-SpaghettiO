package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class AprilTagDetection {
    private int ID;
    private Transform3d transform;
    private double timestamp;
    private static Rotation3d EDN_TO_NWU = new Rotation3d(MatBuilder.fill(Nat.N3(), Nat.N3(), 0, 0, 1, -1, 0, 0, 0, -1, 0));

    public AprilTagDetection(String serial){
        String[] splitSerial = serial.split(" ");

        ID = Integer.parseInt(splitSerial[0]);

        Translation3d trl = new Translation3d(
            Double.parseDouble(splitSerial[1]),
            Double.parseDouble(splitSerial[2]),
            Double.parseDouble(splitSerial[3])
        );

        //7 is at start because qw must be at beginning
        Rotation3d rot = new Rotation3d(
            new Quaternion(
                Double.parseDouble(splitSerial[7]),
                Double.parseDouble(splitSerial[4]),
                Double.parseDouble(splitSerial[5]),
                Double.parseDouble(splitSerial[6])  
            )
        );
        
        //Transfrom must be converted from EDN to NWU
        
        // rot = EDN_TO_NWU.unaryMinus().plus(rot.plus(EDN_TO_NWU));
        // transform = new Transform3d(
        //     trl.rotateBy(EDN_TO_NWU), 
        //     new Rotation3d(rot.getX(), rot.getY(), -rot.getZ())
        // );
        rot = rot.rotateBy(EDN_TO_NWU);
        transform = new Transform3d(
            trl.rotateBy(EDN_TO_NWU),
            rot
        );

        timestamp = Double.parseDouble(splitSerial[8]);
    }

    public int getID() {
        return ID;
    }
    
    public Transform3d getTransform() {
        return transform;
    }

    public double getTimeStamp() {
        return timestamp;
    }
}
