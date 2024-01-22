package frc.robot.DriverSystem;



import com.revrobotics.CANSparkMax;

import java.util.ArrayList;
import java.util.Collections;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//Simülasyonu çalıştırmak için Keyboard Analog importu (Geçici)
import frc.robot.DriverSystem.AdditionalClasses.*;

public  class MotorControllerModule {
     //Bu class bir nevi bizim için bir çok class içerisinde kullanacağımız ve robottaki bir çok kodun temelini oluşturacak bir class. 
     // Bu class içerisindeki işlemler bir nevi robotun içerisindeki temel dinamikler diyebilirim bu temel dinamiklere örnek olarak sürüş, dönme hareket durma ivmeli hareket vs.
     //Bu class'deki fonksiyonların ve  bir çok değişkeninin public olarak tanımlanması veya OOP(Object oriented) mantığı ile encapsulation olarak tanımlanmış olması kodun ilerleyişi açısından daha iyi olur.
    /************************************************** */ 
    //Ek modüllerin tanımlaması
    KeyboardAnalog Key_Analog = new KeyboardAnalog();
    // Motorların tanımlamaları
    private final  CANSparkMax Left_Leader = new CANSparkMax(0,MotorType.kBrushless);
    private final  CANSparkMax Left_Follower= new CANSparkMax(1,MotorType.kBrushless);
     private final  CANSparkMax Right_Leader = new CANSparkMax(2,MotorType.kBrushless);
     private final CANSparkMax Right_Folower = new CANSparkMax(3,MotorType.kBrushless);
     //Ana differential Drive değişkenimiz
     public DifferentialDrive Main_Robot_Drive = new DifferentialDrive(Left_Leader, Right_Leader);
     //Motora ait bazı verilerin Tanımlamaları
     private ArrayList<Double> Speed_Of_Each_Motors;
     // Constructor
     public MotorControllerModule()
     {
        Speed_Of_Each_Motors = new ArrayList<>(Collections.nCopies(2, 0.0));
        Left_Leader.set(0);
        Right_Leader.set(0);
     }
     
     /*|region : PWM MOTOR KONTROL  |*/

     // Periodic Metot // Hem Teleoperation hem de Autonomous için kullanılabilir 
     public void Follow_Periodic()
     {
       Left_Follower.follow(Left_Leader);
       Right_Folower.follow(Right_Leader);
       
     }

     // Periodic Metot //Teleop metotlar için
      public void Teleop_Drive_Periodic(Joystick leftJoystick, Joystick rightJoystick)
      {
           //Burada bu joysticklerdeki axis'lerin itilme miktarını çektik
           if(leftJoystick != null && rightJoystick != null)
           {
            Speed_Of_Each_Motors.set(0, leftJoystick.getY());
            Speed_Of_Each_Motors.set(1, rightJoystick.getY());
           }
           else
           {
             //Eğer ki joystickler tanımlı depilse joystick'in görevlerini geçici süreliğine Klavyeden yazılan tuşlar üstlenecek
             Key_Analog.periodic_KeyListener();
             
             Speed_Of_Each_Motors.set(0,  Key_Analog.Motor_Speed_Key_Analog("Left"));
             Speed_Of_Each_Motors.set(1,  Key_Analog.Motor_Speed_Key_Analog("Right"));
             
           }

          //ARCADE DRIVE ALGORİTMASI
          //* */
          //Arcade Drive sürüş tekniği şu şekildedir. Sürücünün robotu yönetmek için elinde tek bir joystick vardır bu Josytick'deki ilk arduinosu robotun hareketini kontrol edebilmemizi sağlar joystick'deki diğer arduinosu ise robotun ileri ve geri hareketini sağlar
          // burada tek bir joystic olduğu için kontrol olarak Tank Drive'dan daha rahattır ancak daha klasik hareketlere olanak tanır
          //Joystick'de  hem sol motoru  hem de sağ motoru aynı motor olarak varsayıp tek bir motor olarak ayarlamalıyız
          //Arcade drive'da bu yüzden ilk paramatre motora verilen hızı sağdaki parametre ise dönüşü sağlar
         //Main_Robot_Drive.arcadeDrive( Speed_Of_Each_Motors.get(0),Speed_Of_Each_Motors.get(1));

         //TANK DRIVE ALGORİTMASI
         //* */
          // Tank Drive sürüş tekniği şu şekildedir sürücünün robotu yönetmek için kullanacağı iki tane joystic vardır birisi sağ diğeri solu yönetir ve sürücü robotun hareket için bu Josyticklerin itilme miktarına göre robota harektini verir
          // Örneğin robotun ileri gitmesi için iki josytic'de aynı oranda ileri itilmeli sola doğru gitmesi için sağdaki josytick soldakinden daha fazla itilerek sağdaki motorun sola sapması sağlanması
          //* */
          //Robotumuzu tank Drive bir şekilde geliştireceğimiz için bu tanımladığımız Right ve Left Leader'laarın motorlarına verilen bir güç olmalı 
         //Bu gücü de Input Processing Class'i içerisinde tanımlanmış joystick'lerin yön tuşlarındaki kolun ne kadar itildiğine bağlı olarak alacağı değere bağlı olacak. Artı olarak parametrelere değerini Input Processing'deki Joystickler Verecek 
          Main_Robot_Drive.tankDrive( Speed_Of_Each_Motors.get(0),Speed_Of_Each_Motors.get(1));
      }

      //Sensör entegrasyon ve Telemetri modüllerinde bu tarz fonksiyonlardan yararlanarak robotun verilerini alacağız
      public ArrayList<Double> Get_Speed_Of_Each_Motors()
      {
         return  Speed_Of_Each_Motors;
      }
      public ArrayList<CANSparkMax> Get_Leader_Motors()
      {
         ArrayList<CANSparkMax> Can_Spark = new ArrayList<CANSparkMax>();
         Can_Spark.add(Left_Leader);
          Can_Spark.add(Right_Leader);
         return Can_Spark;
      }
       /*|Endregion : PWM MOTOR KONTROL  |*/
}


