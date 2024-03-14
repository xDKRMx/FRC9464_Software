package frc.robot.DriverSystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ManipulationSystem.ShooterModule;
import frc.robot.ManipulationSystem.ShooterModule.AMPmotorStatus;
import frc.robot.ManipulationSystem.ShooterModule.ShooterMotorStatus;

//Simülasyonu bilgisayarda yapabilmek için klavye analog ataması
public  class InputProcessingModule {
     //Bu class Motor Controllerla asenkron bir şekilde geliştirilecek yine içerisinde Robot için temel işlemleri barındıran kısım diyebiliriz
     // Bu class içerisindeki işlemler Input işlemleri olarak düşünülebilir yani Jostick'ten basılan bir tuşun nasıl işlem gereçekleştirileceğini buradan yapacağız
     //NOT: bu class'deki bazı işlemler için MotorControllerModule içerisindeki bazı fonksiyonların oluşturulmuş olması gerekmektedir. Yani eğer ki robotun sürüş algoritması yapılmamışsa buradan Jostick'in axis butonlarından birine basıldığında bu fonksiyona erişelemeyerek hata verir.
     //Bu class'deki fonksiyonların ve bir çok değişkeninin public olarak tanımlanması veya OOP(Object oriented) mantığı ile encapsulation olarak tanımlanmış olması kodun ilerleyişi açısından daha iyi olur.
     /************************************************** */ 
     //Joystick tanımlamaları
     Joystick Joystick = new Joystick(0);
     private int Active_button = -1;
     private int Active_POV = -1;
     //Composition ve Encapsulation mantığı ile modüllerin Class içerisine çağırılıp örneklerinin alınması
     private MotorControllerModule Motor_Controller_Module;
     private ShooterModule Shooter_Module;
     /***************************/ 

      // Constructor
     public InputProcessingModule(MotorControllerModule M_C_Module)
     {
        Motor_Controller_Module = M_C_Module;
        Shooter_Module = Motor_Controller_Module.Shooter_Module;
     }
      /***************************/
      /*|region : JOYSTICK GİRİŞ İŞLEMLERİ |*/
      
      //Periodic Metot // Hem teleop hem de Autonomous için kullanılabilir
       public void Call_Driver_Periodic(boolean Is_It_Teleop)
       {
        // bu fonksiyon içerisinde portlara takılmış olan Joystickleri kullanarak robotun periodic olarak motorlara güç verip hareket etmesini sağlayacağız
        if(Is_It_Teleop) 
        {
         //Driver station üzerinden joysticklerin bağlı olup olmadığını kontrol ediyoruz eğer ki bağlı değilse keyboard üzerinden işlem yapmasını istiyoruz
         if(Connection_Check(Joystick)) Motor_Controller_Module.Teleop_Drive_Periodic(Joystick);
         else Motor_Controller_Module.Teleop_Drive_Periodic(null);
          if(Connection_Check(Joystick))
         {
          //Periyodik olarak butonların basılıp basılmadığına dair kontrolü 
           Joystick_Button_Processing();
           Joystick_POV_Processing();
           //Rotate kontrol
           Rotate_Control();
           //Shoot kontrolü
            Shoot_Note();
         }
        }
      
       }
      

      /*|Endregion : JOYSTICK GİRİŞ İŞLEMLERİ |*/
      /***************************/

      /*| Region : JOYSTICK DÜĞME İŞLEME| */
      public int Joystick_Button_Processing()
      {
          //Left JoystickControl
          if(Joystick.getRawButton(1))   Active_button = 1;
          else if(Joystick.getRawButton(2)) Active_button = 2;
          else if(Joystick.getRawButton(3)) Active_button = 3;
          else if(Joystick.getRawButton(4)) Active_button = 4;
          else if(Joystick.getRawButton(5)) Active_button = 5;
          else if(Joystick.getRawButton(6)) Active_button = 6;
          else if(Joystick.getRawButton(7)) Active_button = 7;
          else if(Joystick.getRawButton(8)) Active_button = 8;
          else if(Joystick.getRawButton(9)) Active_button = 9;
          else if(Joystick.getRawButton(10)) Active_button = 10;
          else if(Joystick.getRawButton(11)) Active_button = 11;
          else if(Joystick.getRawButton(12)) Active_button = 12;
          else if(Joystick.getRawButton(13)) Active_button = 13;
          else if(Joystick.getRawButton(14)) Active_button = 14;
          else if(Joystick.getRawButton(15)) Active_button = 15;
          else if(Joystick.getRawButton(16)) Active_button = 16;
          else Active_button = -1;
          return Active_button;
      }
        public int Joystick_POV_Processing(){
          if(Joystick.getPOV() == 0) Active_POV = 0;
          else if (Joystick.getPOV() == 90 ) Active_POV = 90;
          else if (Joystick.getPOV() == 180) Active_POV = 180;
          else if (Joystick.getPOV() == 270) Active_POV = 270;
          else Active_POV = -1;
          return Active_POV;

        }

        //Rotate (+ için sol 1, - için sağ 3)
        public void Rotate_Control() {
          //Joystick 1'de trigger'ın basılıp basılmamasına göre döndürme
          if(Active_button == 15) Motor_Controller_Module.Rotate_Robot(1,true);
          //Negatif yön
          else if(Active_button == 16)Motor_Controller_Module.Rotate_Robot(1,false);
          else Motor_Controller_Module.Stop_Rotating();

          //Joystick 2'de triggerın basılma oranına göre döndürme
          // double L1_Input = Joystick.getRawAxis(2);
          // double R1_Input = Joystick.getRawAxis(3);
          // // Pozitif yön
          // if(L1_Input > 0.1 && R1_Input < 0.1) Motor_Controller_Module.Rotate_Robot(L1_Input,true);
          // //Negatif yön
          // else if(L1_Input < 0.1 && R1_Input > 0.1)Motor_Controller_Module.Rotate_Robot(R1_Input,false);
          // else Motor_Controller_Module.Stop_Rotating();
        }

        //Shooting Sistemi (L1 için atış AMP'ye, R1 için atış Hopörlöre yapılır) 
        public void Shoot_Note()
        {
          if(Active_button==2) Shooter_Module.Intaking_Note("Shooter");
          else if(Active_POV==90) Shooter_Module.Shoot_Subsystem("Amp","Shooter");
          else if(Active_button==1)  Shooter_Module.Shoot_Subsystem("Speaker","Shooter");
          else if(Active_POV==180)Shooter_Module.Intaking_Note("AMP");
          else if(Active_POV==0) Shooter_Module.Shoot_Subsystem("","AMP");
          
          else if(Active_POV==270) Shooter_Module.Shoot_Subsystem("Speaker" ,"Shooter");
          else
          {
            if(Shooter_Module.Shooter_Status == ShooterMotorStatus.Dynamic) Shooter_Module.SlowDown_Motor_Power("Shooter");
             if(Shooter_Module.AMP_Status == AMPmotorStatus.Dynamic) Shooter_Module.SlowDown_Motor_Power("AMP");
          }
        }

       /*| End Region : JOYSTICK DÜĞME İŞLEME| */
      /***************************/

      /*| Region : HATA YÖNETİMİ VE GÜVENLİK KONTROL| */
       //Joysticklerden gelen sinyalleri işlemek için algoritma
       //Periodic metot
     

      //Joysticklerin ve klavyenin bağlantısını kontrol etmek için algoritma
      //Kontrol metodu
      public boolean Connection_Check(Joystick Joystick)
      {
        // Joysticklerin  bağlantısını kontrol etmek için algoritmanın fonksiyonunu burada çağırın
          boolean Is_Connected = Joystick.isConnected();
          SmartDashboard.putBoolean("Joystick Connected", Is_Connected);
          return Is_Connected;
      }
      /*| End Region : JOYSTICK GİRİŞ İŞLEMLERİ| */
      /***************************/
}