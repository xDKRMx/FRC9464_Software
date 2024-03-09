package frc.robot.ManipulationSystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.DriverSystem.MotorControllerModule;

import com.revrobotics.CANSparkLowLevel.MotorType;

//import frc.robot.DriverSystem.*;
public  class ShooterModule {
     public enum ShooterMotorStatus
     {
       Static,
       Dynamic
     }
     public enum AMPmotorStatus
     {
       Static,
       Dynamic
     }
     //Bu class Driver System artık rayına yavaş yavaş oturduğu zaman geliştirilmeye başlanacak olan class
     //Bu class Intake Modülü ile asenkron bir şekilde geliştirilmeye açık bir class'tir 
     // Bu class üzerinden Intake modülündeki algortimalarla aldığımız notaların hopörlöre atma veya amfiye yerleştirilme işlevi yapılacaktır
     // Burada yapılacak algoritmalar genellikle sapmanın minimalize edilmesi için PIDcontroller kullanılarak oluşturulabilir.
     //Bu clas içerisinde sadece teleop mod için değil aynı zaman autonomous mod için de kod yazmamız gerekecek
     /*****************/
     //Robotun motorlarının Durumu
     //Shooter status
     public ShooterMotorStatus Shooter_Status = ShooterMotorStatus.Static;
     //Amp Status 
     public AMPmotorStatus AMP_Status = AMPmotorStatus.Static;
      //Ek modüllerin tanımlaması
      private MotorControllerModule Motor_Control_Module;
      //Atış yapacak motorların tanımlanması
       private CANSparkMax Shooter_Motor1 = new CANSparkMax(5, MotorType.kBrushless);
       private CANSparkMax Shooter_Motor2 = new CANSparkMax(6, MotorType.kBrushless);
     //AMP kısımı notayı bırakma işlemi
     private CANSparkMax Amp_Motor = new CANSparkMax(7, MotorType.kBrushless); 
     //Amp kısmı için değişkenler
     public Double Cuurent_AMP_Power = 0d;
     //Hangi bölüme atış yapılacağına göre atış gücü ayarlama
      public Double Target_AMP_power = 0.5;
      public double Target_Speaker_Power = 1;
      private Double Current_Shooter_Power = 0d;
      //Atış işlemi için yapılan boolean kontrolleri
      private Boolean is_Shot_Fired;
      public boolean Ready_For_Shooting;
      //Robotun notay sahip olup olmama kontrolü
      public Boolean Is_Note_Entried = true;
     /*********** */
     //Constructor
     public ShooterModule(MotorControllerModule _Motor_Control_Module)
     {
          Motor_Control_Module = _Motor_Control_Module;
          Shooter_Init();
     }
     //Bu modülde ilk başta yapılacak şey notayı robotun motorlarla atma işlemini yapmak
     // Ardından robotun notayı aldığı anda robotun notaya sahip olup olma kontrolünü yapan fonksiyon
     //robotun PID kullanarak robotun notayı AMP'ye göre veya hopörlöre göre atma açısını ayarlaması(robotun kolundaki açıyı notayı atacak şekilde ayarlama)
     void Shooter_Init()
     {
          // Fabrika ayarlarına dön
           /*ROBOTUN DEPLOY EDERKEN SPARK hatası vermemesi için spark max  ile alakalı olan kodlar yorum satırına alınd */
            /****** */
          Shooter_Motor1.restoreFactoryDefaults(); 
          Shooter_Motor2.restoreFactoryDefaults();
          Amp_Motor.restoreFactoryDefaults();
          //Bu boolean Ultrasonic sistem ile veya Touch sensör ile Notanın robotun içine geçtiğinde çalışarak robotun notaya sahip olduğunu belirtir
          //Bu işlem yapılana kadar şimdilik int kısmında boolean değeri true olacaktır
          Ready_For_Shooting = true;
          
     }
     public void Shooter_Periodic()
     {
          Replacing_Note_Control();
     }
     /* | Region : Notayı Çekme Algoritması |*/
     //Robotumuz da Intake sistemi olmadığı için bizim robota notayı koyabilmemizin tek çaresi robotun kaynal noktasına gidip oradan player tarafından içine nota koyulması
     //Biz robotumuzun içerisinde nota olup olmadığı kontrol etmeliyiz aksi takdirde robotumuz boş yere atış motorunu her defasında çağırır
     //Bunu yapmanın iki seçeneği var : // Birincisi robotun notayı aldığını notanın yerleşeceği yerleşke kısmına touch sensörü koyarak nota içeri girdiğinde onu tetiklemek
     //İlk alternatifte oluşabilecek temazsızlık sıkıntılarına çare olarak ikinci alternatifte Ultrasonic koyarak  direkt robotun notayı alıp almadığını sensörden gelen ses dalgalarının notaya çarpmasıyla algılayacağız
     /*Shooter İçin*/
     public void Intaking_Note(String Shooting_Section)
      {
          if(Shooting_Section == "Shooter")
          {
           Shooter_Motor1.set(-0.2);
          Shooter_Motor2.set(-0.2);
           is_Shot_Fired = true;
          Shooter_Status = ShooterMotorStatus.Dynamic;
          }
          else if(Shooting_Section == "AMP") 
          {
                Amp_Motor.set(-0.1);
                AMP_Status = AMPmotorStatus.Dynamic;
          }
      }

      public void Replacing_Note_Control()
      {
       Is_Note_Entried = Motor_Control_Module.Sensor_Integration.Note_Touch_Control();
       // |* İkinci Alternatif *| nota algılama alternatifi Ultrasonic sensörü ile algılama
       //double Note_Distance = Motor_Control_Module.Sensor_Integration.Robot_Get_Distance();
       //Is_Note_Entried = (Note_Distance < 0.3) ? true : false;
       if(Shooter_Status != ShooterMotorStatus.Dynamic)
        Ready_For_Shooting = Is_Note_Entried ? true : false;
      }

     /* | End Region : Notayı Çekme Algoritması |*/


     /* | Region : Notayı shoot etme sistemi  |*/
     //Joystick üzerinden nasılan L1 veya R1 düğmesine göre buradaki atış işlemi gerçekleşecek
     /*Shooter kısımı için */
     public void Shoot_Subsystem(String Shooting_Type, String Shooting_Section)
     {
          //Robotun atabileceği bir nota olup olmadığına göre atış yapılsın
          if(Shooting_Section == "Shooter")
          {
          if(Ready_For_Shooting)
          {
            //Robotun Notaya göre atacak motorlarını hazırlama
                if("Amp".equals(Shooting_Type)) 
              {
                Current_Shooter_Power = Target_AMP_power;
                Shooter_Motor1.set(Target_AMP_power);
                Shooter_Motor2.set(Target_AMP_power);
              } 
              else if("Speaker".equals(Shooting_Type))
              {
                  Current_Shooter_Power = Target_Speaker_Power;
                 Shooter_Motor1.set(Target_Speaker_Power);
                Shooter_Motor2.set(Target_Speaker_Power);
              } 
               is_Shot_Fired = true;
              Shooter_Status = ShooterMotorStatus.Dynamic;
           }
          }
          else if(Shooting_Section == "AMP")
          {
            Amp_Motor.set(0.4);
           Cuurent_AMP_Power = Amp_Motor.get();
            Shooter_Status = ShooterMotorStatus.Dynamic;    
          }
         
     }
     /* | End Region : Notayı shoot etme sistemi  |*/
     /*|Region : Motorları Yavaşlatma | */
     /*Shooter kısımı için */
     //Robot, atış işlemini yaptıktan sonra atışı yapan shooter motorlarını yavaşlatmaya geçme algoritması
     public void SlowDown_Motor_Power(String Shooting_Section)
     {
          Thread thread = new Thread(new Runnable() {
               @Override
               public void run() {
                    //Thread kullanarak bu işlemin asenkron bir şekilde yapılmasını sağlayacağız. Bu sayede robot hareket ederken bir yanda da atış yapıp bir yandan da notayı fırlattıktan sonra motoru durdurabilecek.
                    if(Shooting_Section == "Shooter" && Shooter_Status == ShooterMotorStatus.Dynamic)
                    {
                       while (!Thread.currentThread().isInterrupted()) {
                        Shooter_Status = ShooterMotorStatus.Static;
                        try (// PID denetleyicisi oluşturup buradaki katsayılar üzerinden motorları yavaşlatacağız
                              PIDController pidController1 = new PIDController(0.2, 0.05, 0.01)) {
                             // Setpoint'i 0 olarak ayarlama
                             pidController1.setSetpoint(0.0);
                             if (is_Shot_Fired) {
                              // PID çıktısını hesaplayın
                              double output = pidController1.calculate(Current_Shooter_Power);
                              Current_Shooter_Power =  Current_Shooter_Power > 0 ? Current_Shooter_Power - 0.05 : Current_Shooter_Power + 0.05d;
                              System.out.println(Current_Shooter_Power + " Shooter");
                              //PID çıktısını hem üst hem de alt shooter motor için verdik burada değerleri aynı değişken değerlerini veriyoruz çünkü motorların aynı güçte notayı fırlatıp aynı güçte motorların yavaşlaması lazım
                               Shooter_Motor1.set(output);
                               Shooter_Motor2.set(output);
                              //Eşik değerin altında bir güçte iken motorlar daha fazla bekletmeyip direkt durduruyoruz
                              if (Math.abs(Current_Shooter_Power) < 0.05) {
                                  stopShooting("Shooter");
                                  Thread.currentThread().interrupt();
                              }
                              try {
                                   Thread.sleep(50); // 50 milisaniye bekleme
                               } catch (InterruptedException e) {
                                   Thread.currentThread().interrupt();
                              }
                             }
                        }
                      } 
                    }
                    else if(Shooting_Section == "AMP" && AMP_Status == AMPmotorStatus.Dynamic)
                    {
                        while (!Thread.currentThread().isInterrupted()) {
                        AMP_Status = AMPmotorStatus.Static;
                        try (// PID denetleyicisi oluşturup buradaki katsayılar üzerinden motorları yavaşlatacağız
                              PIDController pidController1 = new PIDController(0.2, 0.05, 0.01)) {
                             // Setpoint'i 0 olarak ayarlama
                             pidController1.setSetpoint(0.0);
                             if (is_Shot_Fired) {
                              // PID çıktısını hesaplayın
                              double output = pidController1.calculate(Cuurent_AMP_Power);
                              Cuurent_AMP_Power =  Cuurent_AMP_Power > 0 ? Cuurent_AMP_Power - 0.05 : Cuurent_AMP_Power + 0.05d;
                              System.out.println(Cuurent_AMP_Power + " Shooter");
                              //PID çıktısını hem üst hem de alt shooter motor için verdik burada değerleri aynı değişken değerlerini veriyoruz çünkü motorların aynı güçte notayı fırlatıp aynı güçte motorların yavaşlaması lazım
                               Amp_Motor.set(output);
                              //Eşik değerin altında bir güçte iken motorlar daha fazla bekletmeyip direkt durduruyoruz
                              if (Math.abs(Cuurent_AMP_Power) < 0.05) {
                                  stopShooting("AMP");
                                  Thread.currentThread().interrupt();
                              }
                              try {
                                   Thread.sleep(50); // 50 milisaniye bekleme
                               } catch (InterruptedException e) {
                                   Thread.currentThread().interrupt();
                              }
                             }
                        }
                      } 
                    }
               }
           });
           thread.start(); // Thread'i başlat
     }

     //atış motorunu sıfırlama işlemi
     
     public void stopShooting(String Shooting_Section) {
          if(Shooting_Section == "Shooter")
          {
             Current_Shooter_Power = 0d;
           Shooter_Motor1.set(0.0);
          Shooter_Motor2.set(0.0);
          is_Shot_Fired = false;
          }
          else if(Shooting_Section == "AMP")
          {
             Cuurent_AMP_Power = 0d;
           Amp_Motor.set(0.0);
          }
     }
     /*|Region : Motorları Yavaşlatma | */
     
}
