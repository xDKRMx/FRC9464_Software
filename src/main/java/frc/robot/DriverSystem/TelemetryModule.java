package frc.robot.DriverSystem;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public  class TelemetryModule {
     //Bu class Sensor entegrasyon class'inin devamı niteliğindedir
      //Not: bu Class'i geliştirmeye başladığımız zaman bir daha MotorControllerModule veya InputProcessingModule'lerine geri dönmeyeceğiz gibi algılanmasın. Burada çakışan kodlar olursa her türlü iyileştirilmeler yapılacaktır
     // Bu kod daha çok autonomous kısmı için önemli bir kod diyebilirim burada sensörlerle ilgili daha karmaşık kodlar ve yapılar yapılacaktır
     //Buradaki kodların işlevi temelde Autonomous kısmı için pürüzsüzlüğü sağlayacak bir class olacak
     //Örnek olarak  sol ve sağ encoderlarından gelen mesafe verilerini kullanarak belirlenen bir hedef mesafeye doğru hareket etmesi buradan sağlanabilir
     //Not : Bu class SensorIntegration'dan çok da farklı bir class olarak düşünülmemelidir burada sadece o class içerisinde kod karmaşası yaşanmaması için bazı kompleks olabilecek kodları buraya yazmaya söz konusudur.
     //Bu class'deki fonksiyonların ve bir çok değişkeninin public olarak tanımlanması veya OOP(Object oriented) mantığı ile encapsulation olarak tanımlanmış olması kodun ilerleyişi açısından daha iyi olur.
      /************************************************** */ 
      private MotorControllerModule _Input_Processing ;
     //Constructor
     public TelemetryModule(MotorControllerModule Input_Processing)
     {
        _Input_Processing  = Input_Processing;
     }
     /*|region : Smart Dashboard İşlemleri|*/
      
      public void Defining_Variables_Smart_Dashboard()
      {
        //Motorların hız değerlerini çekiyoruz
        ArrayList<Double> Speeds = _Input_Processing.Get_Speed_Of_Each_Motors();
        //Aldığımız verileri Smart Dashboard'a yansıtıyoruz
         SmartDashboard.putNumber("Left motor Speed ", Speeds.get(0));
         SmartDashboard.putNumber("Right motor Speed ", Speeds.get(1));
      }
     /*|EndRegion : Smart Dashboard İşlemleri|*/
}
