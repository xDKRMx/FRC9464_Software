package frc.robot.ManipulationSystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public  class VisionProcessing 
{
 //Limelight'ın algıladığı verileri Network tables üzerinden biz buraya aktaracağız
  private static NetworkTable table;
  private static NetworkTableEntry tx;
  private static NetworkTableEntry ty;
  private static NetworkTableEntry tv;
  private static NetworkTableEntry ta;
  private static NetworkTableEntry camMode;
  private static NetworkTableEntry ledMode;

  //enum lar  limelight sensörünün farklı modelarını ve fonkisyonlarını kontrol etmeye yarıyor. içerisinde iki fonksiyondan ilki ledin nasıl çalışacağını belirliyor. İkincisi ise mod verisi çekiyor. 
  private enum LEDMode 
  {
    PIPELINE(0),
    OFF(1),
    BLINK(2),
    ON(3);

    private int value;
    private LEDMode(int value)
    {
      this.value = value;
    }

    private int getValue() {
      return this.value;
    }
  }

  //bunda da aynı şekilde kamera modunda bir belirleyici fonksiyon, iki veri çeken fonkisyon var. 
  private enum CamMode
  {
    VISION(0),
    DRIVER(1);

    private int value;
    private CamMode(int value)
    {
      this.value = modeVal;
    }
    private int getValue() {
      return this.value;
    }
  }

  private VisionProcessing()
  {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx"); // imleç ile hedef arasındaki yatay uzaklık (-29.8 to 29.8 degrees).
    ty = table.getEntry("ty"); // imleç ile hedef arasındaki dikey uzaklık (-24.85 to 24.85 degrees).
    tv = table.getEntry("tv"); // doğru bir hedef görüp görmediği (0 or 1).
    ta = table.getEntry("ta"); // hedef alan (0% of image to 100% of image).
    ledMode = table.getEntry("ledMode"); // sensörün led durumu (0-3).
    camMode = table.getEntry("camMode"); // sensörün çalışma modu (0-1).
  }

  public double getTargetOffsetX() {
    return tx.getDouble(0.0);
  }

  public double getTargetOffsetY() {
    return tx.getDouble(0.0);
  }

  public double getTargetArea() {
    return ta.getDouble(0.0);
  }


  // led modunu ayarlama metodu -çağırırken setLEDMode(LEDMode.ON) tarzında yapılıyor.
  public void setLEDMode(LEDMode mode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode.getValue());
  }
  //

  // kamera modunu ayarlama metodu
  public void setCamMode(CamMode mode) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(mode.getValue());
  }
  //

  // görüntü işlemeyi başlatma fonksiyonu -kamerayı görüntü işlemek için hazırlar-
  public void setModeVision() {
      // Turn on LEDs and set camera for vision processing
      setLEDMode(LEDMode.On);
      setCamMode(CamMode.VisionProcessor);
  }
  //

  // manuel kontrol sağlayan kod
  public void setModeDriver() {
      
      setLEDMode(LEDMode.Off);
      setCamMode(CamMode.DriverCamera);
  }
  //

  //sensörün hangi modda olduğu verisini çekecek fonksiyonlar. 
  private boolean isModeDriver()
  {
    return ledMode.getDouble(0.0) == LEDMode.OFF.modeValue && camMode.getDouble(0.0) == CamMode.DRIVER.modeValue;
  }
  private boolean isModeVision()
  {
    return ledMode.getDouble(0.0) == LEDMode.ON.modeValue && camMode.getDouble(0.0) == CamMode.VISION.modeValue;
  }
  //

  //mod switchleme fonksiyonu
  public void toggleMode()
  {
    if (this.isModeDriver())
    {
      this.setModeVision();
    }
    else if (this.isModeVision())
    {
      this.setModeDriver();
    }
    else
    {
      this.blinkLED();
    }
  }

  //farklı metodların limelight'ın ledlerine erişmesini sağlayacak kodlar.
  public void turnOnLED()
  {
    this.setLEDMode(LEDMode.ON);
  }                 
  public void turnOffLED()
  {
    this.setLEDMode(LEDMode.OFF);
  }
  public void blinkLED()
  {
    this.setLEDMode(LEDMode.BLINK);
  }
}