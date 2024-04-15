package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.LinkedList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionTablesListener {
    private static VisionTablesListener instance;

    private NetworkTableInstance networkTable;
    private NetworkTable visionTable;

    private StringArraySubscriber tag1Sub;
    private StringArraySubscriber tag3Sub;

    // TODO: fanagle left cam measurements from more, and others (if necessary),
    // launcher/right should be working

    private Transform3d launcherCamtoRobot = new Transform3d(new Translation3d(-0.3, 0.02, 0.37),
            new Rotation3d(0, 0, Units.degreesToRadians(-179)));
            //Math.toRadians(146.5)
    // private Transform3d launcherCamtoRobot = new Transform3d(new Translation3d(0.3, 0.02, 0.47),
    //         new Rotation3d(0, Units.degreesToRadians(-33.5), Units.degreesToRadians(-179)));
    //         //Math.toRadians(213.5)
    private Transform3d frontCamToRobot = new Transform3d(new Translation3d(0.315, -0.27, 0.22),
            new Rotation3d(Math.toRadians(26), 0, Math.toRadians(-90)));

    private static LinkedList<AprilTagDetection> launcherDetections = new LinkedList<AprilTagDetection>();
    private static LinkedList<AprilTagDetection> frontDetections = new LinkedList<AprilTagDetection>();

    private boolean launcherVisible;
    private boolean frontVisible;

    private AprilTagFieldLayout tagLayout;

    public VisionTablesListener() {
        networkTable = NetworkTableInstance.getDefault();
        visionTable = networkTable.getTable("Vision");
        tag1Sub = visionTable.getStringArrayTopic("Serialized Tags").subscribe(new String[] {});
        tag3Sub = visionTable.getStringArrayTopic("Serialized Tags 3").subscribe(new String[] {});

        try {
            tagLayout = AprilTagFieldLayout.loadFromResource("2024-crescendo.json");
        } catch (IOException e) {

        }

    }

    public void printDetects() {
        printLauncherDetects();
        printFrontDetects();
    }

    public void printLauncherDetects() {
        SmartDashboard.putBoolean("Launcher Tag", getLauncherDetects());
    }
    public void printFrontDetects() {
        SmartDashboard.putBoolean("Front Tag", getFrontDetects());
    }

    public boolean getLauncherDetects() {
        if (tag1Sub.get().length > 0)
            launcherVisible = true;
        else
            launcherVisible = false;

        return launcherVisible;
    }

    public boolean getFrontDetects() {

        if (tag3Sub.get().length > 0)
            frontVisible = true;
        else
            frontVisible = false;

        return frontVisible;
    }

    public Pose2d[] getLauncherPoses() {
        String[] serializedTags = tag1Sub.get();
        for (String tagSerial : serializedTags)
            launcherDetections.add(new AprilTagDetection(tagSerial));

        if (launcherDetections == null || launcherDetections.size() == 0)
            return new Pose2d[] {};

        Pose2d[] poses = new Pose2d[launcherDetections.size()];
        for (int i = 0; i < poses.length; i++) {
            Pose3d tagFieldPos = tagLayout.getTagPose(launcherDetections.get(i).getID()).get();
            Transform3d robotPos = launcherDetections.get(i).getTransform().plus(launcherCamtoRobot);
            robotPos = new Transform3d(
                    new Translation3d(
                            robotPos.getX(),
                            -robotPos.getY(),
                            -robotPos.getZ()),
                    robotPos.getRotation());
            poses[i] = tagFieldPos.transformBy(robotPos).toPose2d();
        }
        // rightDetections.clear();
        return poses;
    }

    public double[] getLauncherTimeStamps() {
        double[] launcherTimestamps = new double[launcherDetections.size()];

        for (int i = 0; i < launcherDetections.size(); i++)
            launcherTimestamps[i] = launcherDetections.get(i).getTimeStamp();

        return launcherTimestamps;
    }

    public Pose2d[] getFrontPoses() {
        String[] serializedTags = tag3Sub.get();
        for (String tagSerial : serializedTags)
            frontDetections.add(new AprilTagDetection(tagSerial));

        if (frontDetections == null || frontDetections.size() == 0)
            return new Pose2d[] {};

        Pose2d[] poses = new Pose2d[frontDetections.size()];
        for (int i = 0; i < poses.length; i++) {
            Pose3d tagFieldPos = tagLayout.getTagPose(frontDetections.get(i).getID()).get();
            Transform3d robotPos = frontDetections.get(i).getTransform().plus(frontCamToRobot);
            robotPos = new Transform3d(
                    new Translation3d(
                            robotPos.getX(),
                            -robotPos.getY(),
                            -robotPos.getZ()),
                    robotPos.getRotation());
            poses[i] = tagFieldPos.transformBy(robotPos).toPose2d();
        }
        // rightDetections.clear();
        return poses;
    }

    public double[] getFrontTimeStamps() {
        double[] frontTimeStamps = new double[frontDetections.size()];

        for (int i = 0; i < frontDetections.size(); i++)
            frontTimeStamps[i] = frontDetections.get(i).getTimeStamp();

        return frontTimeStamps;
    }

    public void clearLauncherDetections(){
         launcherDetections.clear();
    }

    public void clearRightDetections(){
         frontDetections.clear();
    }

    public static VisionTablesListener getInstance() {
        if (instance == null)
            instance = new VisionTablesListener();
        return instance;
    }
}
