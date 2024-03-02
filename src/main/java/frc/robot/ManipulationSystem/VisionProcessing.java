package frc.robot.ManipulationSystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public  class VisionProcessing 
{
 //Limelight'ın algıladığı verileri Network tables üzerinden biz buraya aktaracağız
   private NetworkTable table;
    NetworkTableEntry tv;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry thor;
    NetworkTableEntry tvert;
  //thor ve tvert bize limelightın algıladığı cismin yatay ve dikey yüksekliğini pixel sayısı olarak veriyor. 
  public VisionProcessing() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    thor = table.getEntry("thor");
    tvert = table.getEntry("tvert");
  }

  public double getHeight(){
    return tvert.getDouble(0.0);
  }
  public double getWidth(){
    return thor.getDouble(0.0);
  }
  public boolean hasValidTarget(){
    return tv.getBoolean(false);
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