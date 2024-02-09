package frc.robot.DriverSystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.RobotController;

import com.kauailabs.navx.frc.AHRS;

      //multithreading sınıfı
//MultiThread object = new MultiThread();
//object.start();
      //ile çağrılabilir.

public  class SensorIntegrationModule  {
      //Bu class Motor Controller ve Input Processing Modüllerindeki işlemlerin devamı niteliğindedir
      //Not: bu Class'i geliştirmeye başladığımız zaman bir daha MotorControllerModule veya InputProcessingModule'lerine geri dönmeyeceğiz gibi algılanmasın. Burada çakışan kodlar olursa her türlü iyileştirilmeler yapılacaktır
     // Bu kodun temel işlevi Robotun sahip olduğu sensörlerin tanımlanması ve bu sensötlerle yapılacak alası basit algoritmaların gereksiz yere Robot.Java üzerinden tanımlanması yerimne burada tanımlanacak
     //Örnek olarak mesela Autonomous için Gyro kullanılarak başlangıç noktasından robota en yakındaki amfiye veya hopörlöre olan ortalama mesafe hesaplanabilir 
     // Bu class bir nevi Sensörler için bir Base olacak yani başka class'lerde özellikle Manıpulation sytem içerisinde bu class'de tanımlanmış olan sensörlerden oldukça yararlanacağız o yüzden bu class diğer class'lere neredeyse kusursuz bir imkan sağlamalı Sensör konusunda
     //Bu class'deki fonksiyonların ve bir çok değişkeninin public olarak tanımlanması veya OOP(Object oriented) mantığı ile encapsulation olarak tanımlanmış olması kodun ilerleyişi açısından daha iyi olur.
       
     //Motorların private olarak burada tanımlanması
     private CANSparkMax Left_Leader;
     private CANSparkMax Right_Leader;
     // kullanılacak Class'lerin örnekleri
     private MotorControllerModule Motor_Controller;
     /*Sensör Tanımlamaları*/
     //Encoder Tanımalaması
      private RelativeEncoder leftEncoder;
     private  RelativeEncoder rightEncoder;
     // navX tanımlaması
     private AHRS ahrs;
     private AnalogInput ultrasonic;
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

      /******************/
     //Constructor
     public SensorIntegrationModule(MotorControllerModule _Motor_Controller)
     {
      Motor_Controller = _Motor_Controller;
      navX_MXP_Init();
      ultrasonic_Init();
     }
     /****************/

    /*| Region : Motor Eşleştirma|*/
   //Bu fonksiyonun işlevi MOTOR KONTROL modülü içerisindeki motorları bu modül içerisinde kullanmak için bir eşleştirme işlemi
   public void Motor_Match()
    {
       Left_Leader = Motor_Controller.GET_SPARKMAX_Motors(0);
      Right_Leader =Motor_Controller.GET_SPARKMAX_Motors(2);
      Encoder_Match();
    }
     /*|End Region : Motor Eşleştirma|*/
    /****************/

     /*| Region : SENSORLERLE TEMEL İŞLEMLERİN VERİLERİNİ ÇEKME|*/
     //Encoder aracılığıyla Robotun Hızını Çekme
    // Encoder sensörü
     public Double[] Get_Motors_Speed()
    {
      Motor_Match();
      // Sensör verilerini oku  
      Double left_motor_speed =  Left_Leader.getEncoder().getVelocity();
      Double right_motor_speed = Right_Leader.getEncoder().getVelocity();
      // Sağ ve sol motor hızlarını ayrı ayrı hesapla
      Double[] speed = new Double[2];
      speed[0] = left_motor_speed;
      speed[1] = right_motor_speed;
      return speed;
   }
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
    //navX-MXP sensörü ile robotun yönün çekme
    //navX-MXP Sensörü
    // Dönme açısını ölçme metodu
    void navX_MXP_Init()
    {
      ahrs = new AHRS(SPI.Port.kMXP); 
      reset_Gyro_Yaw();
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
    //Esas ölçüsünü alma
    while (rotation_angle > 180) rotation_angle -= 360;
    while (rotation_angle < -180) rotation_angle += 360;
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
    public Float[] Three_Axis_Velocity()
    {
      Float veloX = ahrs.getVelocityX();
      Float veloY = ahrs.getVelocityY();
      Float veloZ = ahrs.getVelocityZ();
      Float[] Axis_Velocity = {veloX, veloY, veloZ};
      return Axis_Velocity;
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
    //Ultra Sonic Sensör
    void ultrasonic_Init(){
      ultrasonic = new AnalogInput(0);
      Closer_rate();
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

    public void Closer_rate() {
      Thread thread = new Thread(new Runnable() {
          @Override
          public void run() {
              while (!Thread.currentThread().isInterrupted()) {
                  try {
                      double dist1 = Robot_Get_Distance();
                      Thread.sleep(100); // 100 milisaniye beklet
                      double dist2 = Robot_Get_Distance();
                      double closure_rate;
                      if (dist1 - dist2 < 60) {
                          closure_rate = (dist1 - dist2) / 10;
                      } else {
                          closure_rate = 0;
                      }
  
                      // Kapanma hızını yazdır veya başka bir şekilde kullan
                      System.out.println("Kapanma hızı: " + closure_rate);
  
                      // Burada 100 milisaniye bekleme zaten yapıldığı için ekstra bir Thread.sleep(100) çağrısına gerek yok
                  } catch (InterruptedException e) {
                      System.out.println("Thread kesintiye uğradı, döngü sona eriyor.");
                      Thread.currentThread().interrupt(); // Kesinti durumunu koru
                      break; // Döngüyü sonlandır
                  }
              }
          }
      });
  
      thread.start(); // Thread'i başlat
  }
    /*| END Region : SENSORLERLE TEMEL İŞLEMLERİN VERİLERİNİ ÇEKME|*/
   /***************************/

   /*| Region : SENSORLERLE TEMEL İŞLEMLERİ KOMUTUNU VERME|*/

    /*| END Region : SENSORLERLE TEMEL İŞLEMLERİ KOMUTUNU VERME|*/
    /***************************/

  }
  
