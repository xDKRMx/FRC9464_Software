package frc.robot.DriverSystem;
import javax.swing.text.html.parser.Element;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DriverSystem.MotorControllerModule.RobotStatus;
import frc.robot.ManipulationSystem.ShooterModule;

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
           //PID ile motor kontrolünü sağlayacak fonksiyonun çağırılması
           PID_Motor_Speed();
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
          else Active_button = -1;
          return Active_button;
      }
       //PID çağırma(sağ ikinci düğme)
        public String PID_Motor_Speed(){
          String pow_control_ipm = "";
          if(Active_button==1){
          pow_control_ipm="PID";
          Motor_Controller_Module.robot_Status = RobotStatus.CONSTANTPOWER;
          }
          else{
          pow_control_ipm="Stability";
          }
          Motor_Controller_Module.Motor_Power_Control=pow_control_ipm;
          return Motor_Controller_Module.Motor_Power_Control;
      }

        //Rotate (+ için sol 1, - için sağ 3)
        public void Rotate_Control() {
          double L1_Input = Joystick.getRawAxis(2);
          double R1_Input = Joystick.getRawAxis(3);
          // Pozitif yön
          if(L1_Input > 0.1 && R1_Input < 0.1) Motor_Controller_Module.Rotate_Robot(L1_Input,true);
          //Negatif yön
          else if(L1_Input < 0.1 && R1_Input > 0.1)Motor_Controller_Module.Rotate_Robot(R1_Input,false);
          else Motor_Controller_Module.Stop_Rotating();
        }

        //Shooting Sistemi (L1 için atış AMP'ye, R1 için atış Hopörlöre yapılır) 
        public void Shoot_Note()
        {
          if(Active_button==5) Shooter_Module.Shoot_Subsystem("Amp");
          else if(Active_button==6)  Shooter_Module.Shoot_Subsystem("Speaker");
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