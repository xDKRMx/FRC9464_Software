package frc.robot.DriverSystem.AdditionalClasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.Sendable;

public class Pose2dSendable implements Sendable {
    // Robotun fiziksel özelliklerini ve pozisyonunu saklayacak alanlar
    private Pose2d pose;
    private double wheelbase;
    private double trackwidth;
    private double robotLength;
    private double robotHeight;
    // Field2d nesnesi, simülasyon GUI'sinde robotun konumunu göstermek için
    public Field2d field = new Field2d();
    public static Field2d field2 = new Field2d();
    public static Field2d field3 = new Field2d();
    public Pose2dSendable(Pose2d pose, double wheelbase, double trackwidth, double robotLength, double robotHeight) {
        this.pose = pose;
        this.wheelbase = wheelbase;
        this.trackwidth = trackwidth;
        this.robotLength = robotLength;
        this.robotHeight = robotHeight;
        // SmartDashboard'da Field2d nesnesini göster
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Field2", field2);
        SmartDashboard.putData("Field3", field3);
    }

     public Pose2d GetPose() {
       return pose;
    }
    public void setPose(Pose2d pose) {
        this.pose = pose;
        // Field2d nesnesi ile robotun yeni konumunu güncelle
        field.setRobotPose(pose);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Pose2d");
        builder.addDoubleProperty("x", () -> pose.getX(), null);
        builder.addDoubleProperty("y", () -> pose.getY(), null);
        builder.addDoubleProperty("angle", () -> pose.getRotation().getDegrees(), null);
        // Robotun fiziksel özelliklerini de göstermek için ekstra özellikler ekle
        builder.addDoubleProperty("wheelbase", () -> wheelbase, null);
        builder.addDoubleProperty("trackwidth", () -> trackwidth, null);
        builder.addDoubleProperty("robotLength", () -> robotLength, null);
        builder.addDoubleProperty("robotHeight", () -> robotHeight, null);
    }

    // Field2d nesnesini döndüren bir getter metodu
    public Field2d getField() {
        return field;
    }
}