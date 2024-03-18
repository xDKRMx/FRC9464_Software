package frc.robot.DriverSystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.RobotController;

import com.kauailabs.navx.frc.AHRS;


public  class SensorIntegrationModule  {
     
      //Bu class Motor Controller ve Input Processing Modüllerindeki işlemlerin devamı niteliğindedir
      //Not: bu Class'i geliştirmeye başladığımız zaman bir daha MotorControllerModule veya InputProcessingModule'lerine geri dönmeyeceğiz gibi algılanmasın. Burada çakışan kodlar olursa her türlü iyileştirilmeler yapılacaktır
     // Bu kodun temel işlevi Robotun sahip olduğu sensörlerin tanımlanması ve bu sensötlerle yapılacak alası basit algoritmaların gereksiz yere Robot.Java üzerinden tanımlanması yerimne burada tanımlanacak
     //Örnek olarak mesela Autonomous için Gyro kullanılarak başlangıç noktasından robota en yakındaki amfiye veya hopörlöre olan ortalama mesafe hesaplanabilir 
     // Bu class bir nevi Sensörler için bir Base olacak yani başka class'lerde özellikle Manıpulation sytem içerisinde bu class'de tanımlanmış olan sensörlerden oldukça yararlanacağız o yüzden bu class diğer class'lere neredeyse kusursuz bir imkan sağlamalı Sensör konusunda
     //Bu class'deki fonksiyonların ve bir çok değişkeninin public olarak tanımlanması veya OOP(Object oriented) mantığı ile encapsulation olarak tanımlanmış olması kodun ilerleyişi açısından daha iyi olur.
     
     // kullanılacak ek modüllerin tanımlanması
     private MotorControllerModule Motor_Controller_Module;
     //Motorların private olarak burada tanımlanması
     private CANSparkMax Left_Leader;
     private CANSparkMax Right_Leader;
     /*Sensör Tanımlamaları*/
     //Encoder Tanımalaması
      private RelativeEncoder leftEncoder;
     private  RelativeEncoder rightEncoder;
     // navX tanımlaması
     private AHRS ahrs;
     
      /*Robotun yer değiştirmesini hesaplama kullanılacak değişkenler || POSE2D CİNSİNDEN (ODOMETRİ SİSTEMİ)*/ 
      //POSE 2D DEĞERLERİ
      private Pose2d Current_Robot_Pose;
      //başlangıç noktaları
      private double Default_X_Position = 0f;
      private double Default_Y_Position = 0f;
      private Rotation2d Default_Rotation = null;
      //En son bulunduğu noktalar
      private double Last_X_Position = 999f;
      private double Last_Y_Position = 999f;
      private Rotation2d Last_Rotation = null;
      //Mevcut bulunduğu noktalar
      private double Current_X_Position = 0f;
      private double Current_Y_Position = 0f;
      private Rotation2d Current_Rotation = null;
       //UltraSonic sensör tanımlaması
       private AnalogInput ultrasonic;
       //Touch Sensor
       private DigitalInput touchSensor = new DigitalInput(0);
      /******************/
     //Constructor
     public SensorIntegrationModule(MotorControllerModule _Motor_Controller_Module)
     {
      Motor_Controller_Module = _Motor_Controller_Module;
      navX_MXP_Init();
      ultrasonic_Init();
     }
     /****************/

    /*| Region : Motor Eşleştirma|*/
   //Bu fonksiyonun işlevi MOTOR KONTROL modülü içerisindeki motorları bu modül içerisinde kullanmak için bir eşleştirme işlemi
   public void Motor_Match()
    {
       Left_Leader = Motor_Controller_Module.GET_SPARKMAX_Motors(0);
      Right_Leader =Motor_Controller_Module.GET_SPARKMAX_Motors(2);
      Encoder_Match();
    }
     /*|End Region : Motor Eşleştirma|*/
    /****************/

     /*| Region : SENSORLERLE TEMEL İŞLEMLERİN VERİLERİNİ ÇEKME|*/
     //Encoder aracılığıyla Robotun toplam Alınan yolunu Çekme
    // Encoder sensörü
    public  Double Get_Motors_Overshoot()
    {
      Motor_Match();
      //DİKKAT : BU FONKSİYON ROBOTUN TOPLAM ALINAN YOLUNU HESAPLAR YER DEĞİŞTİRMESİNİ DEĞİL 
      // Sensör verilerini oku
      Double left_motor_position  = leftEncoder.getPosition();
      Double right_motor_position  = rightEncoder.getPosition();
      // Ortalama Alınan yolu hesapla
      return (left_motor_position + right_motor_position) / 2;
    }
    //Bu fonksiyon bizim robotumuzun motorlarında kullanacağımız Encoder'la bu sensör entegrasyonunda kullanacağımız Encoder'ı birbiriyle eşleştiriyor bu sayede Encoder sensörü ile şlem yaparken bir sıkıntı yaşamayacağız
    void Encoder_Match()
    {
        leftEncoder = Left_Leader.getEncoder();
        rightEncoder = Right_Leader.getEncoder();
    }
    /*|Title : Robotun X ve Y eksenlerindeki Yer değiştirmesini hesaplama|*/
    public Pose2d Robot_Init_Position()
    {
      Default_X_Position =ahrs.getDisplacementX();
      Default_Y_Position =ahrs.getDisplacementY();
      Default_Rotation =  new Rotation2d(Math.toRadians(Get_Rotation_Angle()));
      Pose2d Default_Pose = new Pose2d(Current_X_Position, Current_Y_Position, Default_Rotation);
      return Default_Pose;
    }
    public Pose2d Robot_Current_Position()
    {
       Current_X_Position =ahrs.getDisplacementX();
       Current_Y_Position =ahrs.getDisplacementY();
       Current_Rotation =  new Rotation2d(Math.toRadians(Get_Rotation_Angle()));
       Current_Robot_Pose = new Pose2d(Current_X_Position, Current_Y_Position, Current_Rotation);
      return Current_Robot_Pose;
    }
    public Pose2d Position_Difference()
    {
      if(Last_X_Position == 999f && Last_Y_Position == 999f && Last_Rotation == null)
      {
        Last_X_Position = Current_X_Position;
        Last_Y_Position = Current_Y_Position;
        Last_Rotation = Current_Rotation;
        Double DifferenceX = Current_X_Position - Default_X_Position;
        Double DifferenceY = Current_Y_Position - Default_Y_Position;
        double differenceRadians = Current_Rotation.getDegrees() - Default_Rotation.getDegrees();
        Rotation2d  Difference_Rotation =new Rotation2d( Math.toRadians(differenceRadians));
        Pose2d Difference_Pose = new Pose2d(DifferenceX, DifferenceY,Difference_Rotation);
        return Difference_Pose;
      }
      else
      {
        Double DifferenceX = Current_X_Position - Last_X_Position;
        Double DifferenceY = Current_Y_Position - Last_Y_Position;
        double differenceRadians = Current_Rotation.getDegrees() - Last_Rotation.getDegrees();
        Rotation2d  Difference_Rotation =new Rotation2d( Math.toRadians(differenceRadians));
         Pose2d Difference_Pose = new Pose2d(DifferenceX, DifferenceY,Difference_Rotation);
        return Difference_Pose;
      }

    }
    /*|End Title : Robotun X ve Y eksenlerindeki Yer değiştirmesini hesaplama|*/
    //navX-MXP Sensörü
    // Dönme açısını ölçme metodu
    void navX_MXP_Init()
    {
      ahrs = new AHRS(SPI.Port.kMXP); 
      reset_Gyro_Yaw();
      Robot_Init_Position();
    }
    public void reset_Gyro_Yaw(){
      ahrs.reset();
    }
    public Double[] Three_Axis_Rotation()
    {
      //3 eksenli rotasyon hesaplama
      double yaw = ahrs.getYaw(); // Robotun yönünü (yaw) al
      double pitch = ahrs.getPitch(); // Robotun eğimini (pitch) al
      double roll = ahrs.getRoll(); // Robotun yanal eğimini (roll) al
      Double[] Axis_Rotation = {yaw, pitch, roll};
      return Axis_Rotation;

    }
    // Spesifik olarak Yaw ekseninde rotasyonu almayı sağlar
    public double Get_Rotation_Angle()
    {
    double rotation_angle = ahrs.getAngle();
    // rotation_angle %= 2;
    // if(rotation_angle < 0) rotation_angle += 2;
    rotation_angle %= 360;
    if(rotation_angle < 0) rotation_angle += 360;
    rotation_angle = Math.toRadians(rotation_angle);
    return rotation_angle;
    }
    //gyro ivmeyi G biriminde veriyor o yüzden m/s^2 ye çevirmek için 9.8 ile çarpıyoruz
    public Float[] Three_Axis_Acceleration()
    {
      Float accelX = ahrs.getWorldLinearAccelX() * 9.80665f;
      Float accelY = ahrs.getWorldLinearAccelY() * 9.80665f;
      Float accelZ = ahrs.getWorldLinearAccelZ() * 9.80665f;
      Float[] Axis_Acceleration = {accelX, accelY, accelZ};
      return Axis_Acceleration;
    }
    //navx velocity i m/s olarak veriyor (navx sayfası bu method deneysel diyor o yüzden düzgün çalışmayabilir.)
    public Double Get_Velocity()
    {
      Float veloX = ahrs.getVelocityX();
      Float veloY = ahrs.getVelocityY();
      Double Absolute_Velocity = Math.sqrt( (veloX*veloX) + (veloY*veloY));
      return Absolute_Velocity;
    }
      public Float get_Height(){
      return ahrs.getDisplacementZ();
    }
    public float get_Magnetic_Heading(){
      return ahrs.getCompassHeading();
    }
    public boolean is_Robot_Moving(){
      return ahrs.isMoving();
    }
    public boolean is_Robot_Rotating(){
      return ahrs.isRotating();
    }
    public void Reset_Displacement(){
      ahrs.resetDisplacement();
    }
  
    //Ultra Sonic Sensör
    void ultrasonic_Init(){
      ultrasonic = new AnalogInput(0);
    }
    public Double Robot_Get_Distance()
    {  
      double raw_Value = ultrasonic.getValue();
      //bu kısmın çoğunu şu siteden aldım https://maxbotix.com/blogs/blog/firstrobotics
      //voltage_scale_factor bizim voltajda herhangi bir değişiklik olması durumunda bile doğru veri almamızı sağlar(0 = 0V, 4095 = 5V değerleri arasında).
      //cm cinsinden birim
      double voltage_scale_factor = 5/RobotController.getVoltage5V();
      return raw_Value * voltage_scale_factor * 0.125;   
    }

    //Touch Sensürünü kullanarak notanın robota yerleşke bölümüne girip girmediğinin kontrolünü yapıyoruz.
    //Eğer ki robotun sahip olduğu bir nota varsa buradaki metodu çağırdığımız zaman true dönüyor eğer ki yoksa false dönüyor
    //Eğer ki true dönmüşse robotun notayı fırlatmak için hazır olduğunu algılıyoruz
    public boolean Note_Touch_Control()
    {
      //Notanın algılanması için dijital sensöre bağladığımız touch sensöründen veri çekiyoruz
      boolean Touch_Control =  touchSensor.get();
      return !Touch_Control;
    }
    /*| END Region : SENSORLERLE TEMEL İŞLEMLERİN VERİLERİNİ ÇEKME|*/
   /***************************/

  }
  
