package frc.robot.DriverSystem.AdditionalClasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Pose2dSendable implements Sendable {
    
    
    private Pose2d pose;
    private double wheelbase;
    private double trackwidth;
    private double robotLength;
    private double robotHeight;

    public Pose2dSendable(Pose2d pose, double wheelbase, double trackwidth, double robotLength, double robotHeight) {
        super();
        this.pose = pose;
        this.wheelbase = wheelbase;
        this.trackwidth = trackwidth;
        this.robotLength = robotLength;
        this.robotHeight = robotHeight;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Pose2d");
        builder.addDoubleProperty("x", () -> pose.getX(), null);
        builder.addDoubleProperty("y", () -> pose.getY(), null);
        builder.addDoubleProperty("angle", () -> pose.getRotation().getDegrees(), null);
        builder.addDoubleProperty("wheelbase", () -> wheelbase, null);
        builder.addDoubleProperty("trackwidth", () -> trackwidth, null);
        builder.addDoubleProperty("robotLength", () -> robotLength, null);
        builder.addDoubleProperty("robotHeight", () -> robotHeight, null);
    }
}