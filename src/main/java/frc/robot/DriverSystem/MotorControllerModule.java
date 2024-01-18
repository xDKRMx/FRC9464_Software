package frc.robot.DriverSystem;

import com.revrobotics.CANSparkMax;

import java.util.ArrayList;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
public  class MotorControllerModule {
     //Bu class bir nevi bizim için bir çok class içerisinde kullanacağımız ve robottaki bir çok kodun temelini oluşturacak bir class. 
     // Bu class içerisindeki işlemler bir nevi robotun içerisindeki temel dinamikler diyebilirim bu temel dinamiklere örnek olarak sürüş, dönme hareket durma ivmeli hareket vs.
     //Bu class'deki fonksiyonların ve  bir çok değişkeninin public olarak tanımlanması veya OOP(Object oriented) mantığı ile encapsulation olarak tanımlanmış olması kodun ilerleyişi açısından daha iyi olur.
    /************************************************** */ 
    // Motorların tanımlamaları

    private final  CANSparkMax Left_Leader = new CANSparkMax(0,MotorType.kBrushless);
    private final  CANSparkMax Left_Follower= new CANSparkMax(1,MotorType.kBrushless);
     private final  CANSparkMax Right_Leader = new CANSparkMax(2,MotorType.kBrushless);
     private final CANSparkMax Right_Folower = new CANSparkMax(3,MotorType.kBrushless);
     //Ana differential Drive değişkenimiz
     public DifferentialDrive Main_Robot_Drive = new DifferentialDrive(Left_Leader, Right_Leader);
     //Motora ait bazı verilerin Tanımlamaları
     private ArrayList<Double> Speed_Of_Each_Motors = new ArrayList<Double>();
     // Constructor
     public MotorControllerModule()
     {
        
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
           Speed_Of_Each_Motors.set(0, leftJoystick.getY());
          
           Speed_Of_Each_Motors.set(1, rightJoystick.getY());
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

       /*|Endregion : PWM MOTOR KONTROL  |*/
}


