package frc.robot.DriverSystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
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
     // Gyro tanımlaması
     private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
      // IMU tanımlaması
      /***************** */
     //Constructor
     public SensorIntegrationModule(MotorControllerModule _Motor_Controller)
     {
      Motor_Controller = _Motor_Controller;
     }

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
     //Encoder aracılığıyla Robotun pozisyonunu Çekme
    // Encoder sensörü
    public  Double[] Get_Motors_Direction()
    {
      // Sensör verilerini oku
     
      Double left_motor_position  = leftEncoder.getPosition();
      Double right_motor_position  = rightEncoder.getPosition();
  
      // Ortalama yön değerini hesapla
      Double[] Directions = new Double[2];
      Directions[0] = left_motor_position;
      Directions[1] = right_motor_position;
      return Directions;
    }

    //Bu fonksiyonun işlevi MOTOR KONTROL modülü içerisindeki motorları bu modül içerisinde kullanmak için bir eşleştirme işlemi
   public void Motor_Match()
    {
       Left_Leader = Motor_Controller.GET_SPARKMAX_Motors(0);
      Right_Leader =Motor_Controller.GET_SPARKMAX_Motors(2);
      Encoder_Match();
    }
    //Bu fonksiyon bizim robotumuzun motorlarında kullanacağımız Encoder'la bu sensör entegrasyonunda kullanacağımız Encoder'ı birbiriyle eşleştiriyor bu sayede Encoder sensörü ile şlem yaparken bir sıkıntı yaşamayacağız
    void Encoder_Match()
    {
       leftEncoder = Left_Leader.getEncoder();
       rightEncoder = Right_Leader.getEncoder();
       
    }

    //Gyro sensörü ile robotun yönün çekme
    //Gyro Sensörü
    // Dönme açısını ölçme metodu
  public double Get_Robot_Angle()
  {
    gyro.calibrate();
    // Gyro sensörü kullanarak robotun dönüş açısını ölç
    double angle = gyro.getAngle();

    return angle;
  }
    /*| END Region : SENSORLERLE TEMEL İŞLEMLERİN VERİLERİNİ ÇEKME|*/


   /*| Region : SENSORLERLE TEMEL İŞLEMLERİ KOMUTUNU VERME|*/

    /*| END Region : SENSORLERLE TEMEL İŞLEMLERİ KOMUTUNU VERME|*/
}
