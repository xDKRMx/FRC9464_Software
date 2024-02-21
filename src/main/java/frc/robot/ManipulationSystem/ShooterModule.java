package frc.robot.ManipulationSystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;

//import frc.robot.DriverSystem.*;
public  class ShooterModule {
     //Bu class Driver System artık rayına yavaş yavaş oturduğu zaman geliştirilmeye başlanacak olan class
     //Bu class Intake Modülü ile asenkron bir şekilde geliştirilmeye açık bir class'tir 
     // Bu class üzerinden Intake modülündeki algortimalarla aldığımız notaların hopörlöre atma veya amfiye yerleştirilme işlevi yapılacaktır
     // Burada yapılacak algoritmalar genellikle sapmanın minimalize edilmesi için PIDcontroller kullanılarak oluşturulabilir.
     //Bu clas içerisinde sadece teleop mod için değil aynı zaman autonomous mod için de kod yazmamız gerekecek
     /*****************/
      //Ek modüllerin tanımlaması
      //Atış yapacak motorların tanımlanması
      private CANSparkMax Shooter_Motor = new CANSparkMax(4, MotorType.kBrushless);
     //Hangi bölüme atış yapılacağına göre atış gücü ayarlama
      public Double Target_AMP_power = 0.4;
      public double Target_Speaker_Power = 1;
      private Double Current_Shooter_Power = 0d;
      //Atış işlemi için yapılan boolean kontrolleri
      private Boolean is_Shot_Fired;
      public boolean Ready_For_Shooting;
     /*********** */
     //Constructor
     public ShooterModule()
     {
          Shooter_Init();
     }
     //Bu modülde ilk başta yapılacak şey notayı robotun motorlarla atma işlemini yapmak
     // Ardından robotun notayı aldığı anda robotun notaya sahip olup olma kontrolünü yapan fonksiyon
     //robotun PID kullanarak robotun notayı AMP'ye göre veya hopörlöre göre atma açısını ayarlaması(robotun kolundaki açıyı notayı atacak şekilde ayarlama)
     void Shooter_Init()
     {
          Shooter_Motor.restoreFactoryDefaults(); // Fabrika ayarlarına dön
          Shooter_Motor.setSmartCurrentLimit(40); // Akım limitini ayarla (örnek olarak 40A)
          //Bu boolean Ultrasonic sistem ile veya Touch sensör ile Notanın robotun içine geçtiğinde çalışarak robotun notaya sahip olduğunu belirtir
          //Bu işlem yapılana kadar şimdilik int kısmında boolean değeri true olacaktır
          Ready_For_Shooting = true;
          
     }
     public void Shooter_Periodic()
     {
         
          SmartDashboard.putNumber("Shooter Motor power : ", Current_Shooter_Power);
     }
     //Joystick üzerinden nasılan L1 veya R1 düğmesine göre buradaki atış işlemi gerçekleşecek
     public void Shoot_Subsystem(String Shooting_Type )
     {
          //Robotun atabileceği bir nota olup olmadığına göre atış yapılsın
          if(Ready_For_Shooting)
          {
            //Robotun Notaya göre atacak motorlarını hazırlama
            if("Amp".equals(Shooting_Type))   Shooter_Motor.set(Target_AMP_power);
            else if("Speaker".equals(Shooting_Type)) Shooter_Motor.set(Target_Speaker_Power);
             is_Shot_Fired = true;
            // Ready_For_Shooting = false;
            SlowDown_Motor_Power();
            //Thread Executer  ile motorun gücü eşik değerin altına inene kadar sürekli olarak azaltılıyor
          //   executorService.scheduleAtFixedRate(this::SlowDown_Motor_Power, 0, 100, TimeUnit.MILLISECONDS);
          }
     }
     //Robot, atış işlemini yaptıktan sonra atışı yapan shooter motorlarını yavaşlatmaya geçme algoritması
     public void SlowDown_Motor_Power()
     {
          Thread thread = new Thread(new Runnable() {
               @Override
               public void run() {
                   while (!Thread.currentThread().isInterrupted()) {
                           Current_Shooter_Power = Shooter_Motor.get();
                         if (is_Shot_Fired) {
                              Current_Shooter_Power =  Current_Shooter_Power > 0 ? Current_Shooter_Power - 0.05 : Current_Shooter_Power + 0.05d;
                              Shooter_Motor.set(Current_Shooter_Power);
                              if (Math.abs(Current_Shooter_Power) < 0.05) {
                                  stopShooting();
                                  Thread.currentThread().interrupt();
                              }
                              try {
                                   Thread.sleep(50); // 50 milisaniye bekleme
                               } catch (InterruptedException e) {
                                   System.out.println(e);
                               }
                          }
                   }
               }
           });
       
           thread.start(); // Thread'i başlat
           
     }
     //atış motorunu sıfırlama işlemi
     public void stopShooting() {
          Current_Shooter_Power = 0d;
          Shooter_Motor.set(0.0);
          is_Shot_Fired = false;
      }
}
