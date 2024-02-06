package frc.robot.DriverSystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Ultrasonic;

import com.kauailabs.navx.frc.AHRS;


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
      /*Robotun yer değiştirmesini hesaplama kullanılacak değişkenler */
      //başlangıç noktaları
      private double Default_X_Position = 0f;
      private double Default_Y_Position = 0f;
      private double Default_Rotation = 0d;
      //En son bulunduğu noktalar
      private double Last_X_Position = 999f;
      private double Last_Y_Position = 999f;
      private double Last_Rotation = 999d;
      //Mevcut bulunduğu noktalar
      private double Current_X_Position = 0f;
      private double Current_Y_Position = 0f;
       private double Current_Rotation = 0d;
       //UltraSonic sensör tanımlaması
       Ultrasonic Ultra_Sonic = new Ultrasonic(1,2);
      /******************/
     //Constructor
     public SensorIntegrationModule(MotorControllerModule _Motor_Controller)
     {
      Motor_Controller = _Motor_Controller;
      navX_MXP_Init();
      
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
    public Double[] Robot_Init_Position()
    {
      Default_X_Position =ahrs.getDisplacementX();
      Default_Y_Position =ahrs.getDisplacementY();
      Double[] Virtual_Pose2D = {Default_X_Position,Default_Y_Position};
      return Virtual_Pose2D;
    }
    public Double[] Robot_Current_Position()
    {
       Current_X_Position =ahrs.getDisplacementX();
       Current_Y_Position =ahrs.getDisplacementY();
       Double[] Virtual_Pose2D = {Current_X_Position,Current_Y_Position};
      return Virtual_Pose2D;
    }
    Double[] Position_Difference()
    {
      if(Last_X_Position == 999f && Last_Y_Position == 999f && Last_Rotation == 999f)
      {
        Last_X_Position = Current_X_Position;
        Last_Y_Position = Current_Y_Position;
        Last_Rotation = Current_Rotation;
        Double DifferenceX = Current_X_Position - Default_X_Position;
        Double DifferenceY = Current_Y_Position - Default_Y_Position;
        Double DifferenceRotation = Current_Rotation - Default_Rotation;
        Double[] Differences = {DifferenceX,DifferenceY,DifferenceRotation };
        return Differences;
      }
      else
      {
        Double DifferenceX = Current_X_Position - Last_X_Position;
        Double DifferenceY = Current_Y_Position - Last_Y_Position;
        Double DifferenceRotation = Current_Rotation - Last_Rotation;
        Double[] Differences = {DifferenceX,DifferenceY,DifferenceRotation };
        return Differences;
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
    
    /*| END Region : SENSORLERLE TEMEL İŞLEMLERİN VERİLERİNİ ÇEKME|*/
     /***************************/

   /*| Region : SENSORLERLE TEMEL İŞLEMLERİ KOMUTUNU VERME|*/

    /*| END Region : SENSORLERLE TEMEL İŞLEMLERİ KOMUTUNU VERME|*/
    /***************************/
}
