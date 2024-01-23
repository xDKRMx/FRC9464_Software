package frc.robot.DriverSystem;

import javax.swing.text.html.parser.Element;

import edu.wpi.first.wpilibj.Joystick;

//Simülasyonu bilgisayarda yapabilmek için klavye analog ataması
import frc.robot.DriverSystem.AdditionalClasses.KeyboardAnalog;
public  class InputProcessingModule {
     //Bu class Motor Controllerla asenkron bir şekilde geliştirilecek yine içerisinde Robot için temel işlemleri barındıran kısım diyebiliriz
     // Bu class içerisindeki işlemler Input işlemleri olarak düşünülebilir yani Jostick'ten basılan bir tuşun nasıl işlem gereçekleştirileceğini buradan yapacağız
     //NOT: bu class'deki bazı işlemler için MotorControllerModule içerisindeki bazı fonksiyonların oluşturulmuş olması gerekmektedir. Yani eğer ki robotun sürüş algoritması yapılmamışsa buradan Jostick'in axis butonlarından birine basıldığında bu fonksiyona erişelemeyerek hata verir.
     //Bu class'deki fonksiyonların ve bir çok değişkeninin public olarak tanımlanması veya OOP(Object oriented) mantığı ile encapsulation olarak tanımlanmış olması kodun ilerleyişi açısından daha iyi olur.
     /************************************************** */ 
     //Joystick tanımlamaları
     Joystick Left_Joystick = new Joystick(0);
     Joystick Right_Joystick = new Joystick(1); 
     private int Active_left_button = -1;
     private int Active_right_button = -1;
     private 
     // Klavye Analog Tanımlaması 
     KeyboardAnalog Add_Keyboard = new KeyboardAnalog();
     //Composition ve Encapsulation mantığı ile modüllerin Class içerisine çağırılıp örneklerinin alınması
     private MotorControllerModule Motor_Controller_Module;
     /***************************/ 
      // Constructor
     public InputProcessingModule(MotorControllerModule M_C_Module)
     {
        Motor_Controller_Module = M_C_Module;
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
         if(Connection_Check(Left_Joystick,Right_Joystick)) Motor_Controller_Module.Teleop_Drive_Periodic(Right_Joystick, Left_Joystick);
         else Motor_Controller_Module.Teleop_Drive_Periodic(null,null);

        }
        else
        {
             //Autonomous kısmı 
        }

       }

      /*|Endregion : JOYSTICK GİRİŞ İŞLEMLERİ |*/
      /***************************/
      /*| Region : JOYSTICK DÜĞME İŞLEME| */
      public int[] Joystick_Button_Processing()
      {
          //Left JoystickControl
          if(Left_Joystick.getRawButton(1))   Active_left_button = 1;
          else if(Left_Joystick.getRawButton(2)) Active_left_button = 2;
          else if(Left_Joystick.getRawButton(3)) Active_left_button = 3;
          else if(Left_Joystick.getRawButton(4)) Active_left_button = 4;
          else Active_left_button = -1;
          if(Right_Joystick.getRawButton(1))   Active_right_button = 1;
          else if(Right_Joystick.getRawButton(2)) Active_right_button = 2;
          else if(Right_Joystick.getRawButton(3)) Active_right_button = 3;
          else if(Right_Joystick.getRawButton(4)) Active_right_button = 4;
          else Active_right_button = -1;
          int[] Button_value_list = {Active_left_button, Active_right_button};
          return Button_value_list;
      }

       /*| End Region : JOYSTICK DÜĞME İŞLEME| */
      /***************************/
      /*| Region : HATA YÖNETİMİ VE GÜVENLİK KONTROL| */
       //Joysticklerden gelen sinyalleri işlemek için algoritma
       //Periodic metot
     

      //Joysticklerin ve klavyenin bağlantısını kontrol etmek için algoritma
      //Kontrol metodu
      public boolean Connection_Check(Joystick Left, Joystick Right)
      {
        // Joysticklerin  bağlantısını kontrol etmek için algoritmanın fonksiyonunu burada çağırın
          boolean Is_Connected = Left.isConnected() && Right.isConnected();
          return Is_Connected;
      }
      /*| Region : JOYSTICK GİRİŞ İŞLEMLERİ| */
}