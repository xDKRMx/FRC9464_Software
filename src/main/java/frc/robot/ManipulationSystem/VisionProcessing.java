package frc.robot.ManipulationSystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public  class VisionProcessing 
{
 //Limelight'ın algıladığı verileri Network tables üzerinden biz buraya aktaracağız
   private NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
  public VisionProcessing() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  public double getX() {
    return tx.getDouble(0.0);
  }

  public double getY() {
    return tx.getDouble(0.0);
  }

  public double getArea() {
    return ta.getDouble(0.0);
  }
}