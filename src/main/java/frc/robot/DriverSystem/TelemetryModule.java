package frc.robot.DriverSystem;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// ek class'Lerin tanımalanması için 
import frc.robot.DriverSystem.AdditionalClasses.*;

public  class TelemetryModule {
     //Bu class Sensor entegrasyon class'inin devamı niteliğindedir
      //Not: bu Class'i geliştirmeye başladığımız zaman bir daha MotorControllerModule veya InputProcessingModule'lerine geri dönmeyeceğiz gibi algılanmasın. Burada çakışan kodlar olursa her türlü iyileştirilmeler yapılacaktır
     // Bu kod daha çok autonomous kısmı için önemli bir kod diyebilirim burada sensörlerle ilgili daha karmaşık kodlar ve yapılar yapılacaktır
     //Buradaki kodların işlevi temelde Autonomous kısmı için pürüzsüzlüğü sağlayacak bir class olacak
     //Örnek olarak  sol ve sağ encoderlarından gelen mesafe verilerini kullanarak belirlenen bir hedef mesafeye doğru hareket etmesi buradan sağlanabilir
     //Not : Bu class SensorIntegration'dan çok da farklı bir class olarak düşünülmemelidir burada sadece o class içerisinde kod karmaşası yaşanmaması için bazı kompleks olabilecek kodları buraya yazmaya söz konusudur.
     //Bu class'deki fonksiyonların ve bir çok değişkeninin public olarak tanımlanması veya OOP(Object oriented) mantığı ile encapsulation olarak tanımlanmış olması kodun ilerleyişi açısından daha iyi olur.
      /************************************************** */ 
      private MotorControllerModule _Motor_Controller ;
      // Smart Dashboard'da görüntülenebilmeesi için örnek sanal robot örneği
      // private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.6); // 0.6 metre tekerlek mesafesi
      private DifferentialDrivetrainSim drivetrainSimulator;
      public static Pose2dSendable Pose_Sendable ;
      Pose2d newPose;
     //Constructor
     public TelemetryModule(MotorControllerModule Motor_Controller)
     {
        _Motor_Controller  = Motor_Controller;
        Defining_Variables_Smart_Dashboard();
     }
       /***************************/
     /*|region : Smart Dashboard İşlemleri|*/
      public void Updating_Variables_Smart_Dashboard()
      {
        // Motor çıkışlarını simülasyon modeline uygula
         ArrayList<Double> Leader_Motors = _Motor_Controller.Get_Power_Of_Each_Motors() ;
        drivetrainSimulator.setInputs(Leader_Motors.get(0) * RobotController.getInputVoltage(), Leader_Motors.get(1) * RobotController.getInputVoltage()); // 12.0, motorların maksimum voltajını temsil eder.
        // Simülasyonu güncelle
        drivetrainSimulator.update(0.02); // 20 ms zaman adımı
        // Simülasyondan alınan robot pozisyonu SmartDashboard'a gönderilir.
        //Motorların hız değerlerini çekiyoruz
         ArrayList<Double> Powers = _Motor_Controller.Get_Power_Of_Each_Motors();
          //Aldığımız verileri Smart Dashboard'a yansıtıyoruz
         SmartDashboard.putNumber("Left motor POWER ", Powers.get(0));
         SmartDashboard.putNumber("Right motor POWER ", Powers.get(1));

         // ROBOTUN VAROLAN HIZI
         //  Double[] Speeds = _Motor_Controller.Sensor_Integration.Get_Motors_Speed();
         //   SmartDashboard.putNumber("Left motor SPEED ", Speeds[0]);
         // SmartDashboard.putNumber("Right motor SPEED ", Speeds[1]);
         // Robotun anlık pozisyonunu al
         newPose = drivetrainSimulator.getPose();
         Pose_Sendable.setPose(newPose);
          SmartDashboard.putData("RobotPose", Pose_Sendable);
         // ROBOTUN VAROLAN açısı
         _Motor_Controller.Sensor_Integration.Get_Rotation_Angle();
      }
      public void Defining_Variables_Smart_Dashboard()
      {
         //SmartDashBoard'da girilecek değerleri tanımlıyoruz
         // SmartDashboard.putNumber("Left motor Speed ", 0.0d);
         // SmartDashboard.putNumber("Right motor Speed ", 0.0d);
         //
         //BURADAKİ VERİLER GEÇİCİDİR BU VERİLER SENSÖR ENTEGRASYONUNDAKİ SENSÖRLERDEKİ DEĞERLER GİRİLENE KADAR TEMSİLİ OLARAK YANSITIR
             Pose2d initialPose = new Pose2d(0, 0, new Rotation2d());
             Pose_Sendable = new Pose2dSendable(initialPose,0.6d,0.6d,1d,0.6d);
             SmartDashboard.putData("RobotPose", Pose_Sendable);
            _Motor_Controller.Sensor_Integration.Motor_Match();
            final double KvLinear = 2.0; // volt seg^-1 başına metre (sürüş hızı için tipik bir FRC robot değeri)
            final double KaLinear = 0.8; // volt seg^-2 başına metre (sürüş ivmesi için tipik bir FRC robot değeri)
            final double KvAngular = 1.5; // volt seg^-1 başına radyan (dönüş hızı için tipik bir FRC robot değeri)
            final double KaAngular = 0.7;
            drivetrainSimulator = new DifferentialDrivetrainSim(
            LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular), // Bu değerler robotunuzun gerçek sürüş karakteristiklerine göre ayarlanmalıdır.
            DCMotor.getNEO(2), // Her bir taraf için 2 Falcon 500 motor varsayalım.
            7.29, // Dişli oranı
            7.5, // Dönme kütlesi kg*m^2
            0.0762, // Tekerlek çapı (metre)
            null // Şasi ağırlığı varsayılan olarak alınabilir, ya da belirli bir ağırlık belirtilebilir.
         );
      }
      
     /*|EndRegion : Smart Dashboard İşlemleri|*/
     /***************************/
}
