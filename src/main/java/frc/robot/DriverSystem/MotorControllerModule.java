package frc.robot.DriverSystem;



import com.revrobotics.CANSparkMax;

import java.util.ArrayList;
import java.util.Collections;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//Simülasyonu çalıştırmak için Keyboard Analog importu (Geçici)
import frc.robot.DriverSystem.AdditionalClasses.*;

//BU ENUM yapısı robotun motor durum kontrolünde kullanılacaktır ve ilerleyen aşamalarda sensör entegrasyonundaki veirlerle birlikte sağlanan değerler enum'daki robotun durumna göre şekillenecektir
 enum RobotStatus {
  DYNAMIC,
  IDLE,
  TURNING
}
public  class MotorControllerModule {
     //Bu class bir nevi bizim için bir çok class içerisinde kullanacağımız ve robottaki bir çok kodun temelini oluşturacak bir class. 
     // Bu class içerisindeki işlemler bir nevi robotun içerisindeki temel dinamikler diyebilirim bu temel dinamiklere örnek olarak sürüş, dönme hareket durma ivmeli hareket vs.
     //Bu class'deki fonksiyonların ve  bir çok değişkeninin public olarak tanımlanması veya OOP(Object oriented) mantığı ile encapsulation olarak tanımlanmış olması kodun ilerleyişi açısından daha iyi olur.
    /************************************************** */ 
    //Ek modüllerin tanımlaması
    KeyboardAnalog Key_Analog = new KeyboardAnalog();
    SensorIntegrationModule Sensor_Integration = new SensorIntegrationModule();
    //Robotun durumu
    public RobotStatus robot_Status;
    // Motorların tanımlamaları
    private final  CANSparkMax Left_Leader = new CANSparkMax(0,MotorType.kBrushless);
    private final  CANSparkMax Left_Follower= new CANSparkMax(1,MotorType.kBrushless);
     private final  CANSparkMax Right_Leader = new CANSparkMax(2,MotorType.kBrushless);
     private final CANSparkMax Right_Folower = new CANSparkMax(3,MotorType.kBrushless);
     //Ana differential Drive değişkenimiz
     public DifferentialDrive Main_Robot_Drive = new DifferentialDrive(Left_Leader, Right_Leader);
     //Motora ait bazı verilerin Tanımlamaları
     private ArrayList<Double> Speed_Of_Each_Motors;
     
     // PID değişkenlerinin tanımlamaları
     private double previous_error = 0.0;
     private double integral_sum = 0.0;
     private double dt = 0.01;
     private ArrayList<Double> PID_Coefficients;
     /**********************************/
     // Constructor
     public MotorControllerModule()
     {
        Speed_Of_Each_Motors = new ArrayList<>(Collections.nCopies(2, 0.0));
        Left_Leader.set(0);
        Right_Leader.set(0);
        
     }
     /***************************/
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
            // Mevcut motor hızlarını al
             double currentLeftSpeed = Speed_Of_Each_Motors.get(0);
             double currentRightSpeed = Speed_Of_Each_Motors.get(1);

             // Joystick inputlarını al veya varsayılan olarak 0 kabul et
             double leftJoystickInput = leftJoystick.getY();
             double rightJoystickInput =  rightJoystick.getY();
             // Ramp algoritmasına göre Stabiliteyi sağlayacak bir ayarlama işlemi
             Double Absolute_Left_Motor_Speed = rampMotorInput(currentLeftSpeed,leftJoystickInput,0.1f);
             Double Absolute_Right_Motor_Speed = rampMotorInput(currentRightSpeed,rightJoystickInput,0.1f);
            Speed_Of_Each_Motors.set(0, Absolute_Left_Motor_Speed);
            Speed_Of_Each_Motors.set(1,Absolute_Right_Motor_Speed );
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
         Motor_Stability(0.5d);
          Main_Robot_Drive.tankDrive( Speed_Of_Each_Motors.get(0),Speed_Of_Each_Motors.get(1));
      }
       //Motorlara verilen güçlerin stabilitesi ve max motor gücü farkı
       void Motor_Stability(Double max_Difference)
       {
         double Current_Difference = Math.abs(Speed_Of_Each_Motors.get(0) - Speed_Of_Each_Motors.get(1));
        // Current Difference Control
        //İLK ALTERNATİF
         while(Current_Difference > max_Difference)
         {

            Current_Difference /= 2;
            if(Current_Difference <= max_Difference)
           {
            Double Half_difference = Current_Difference /2;
            Boolean extensial = Speed_Of_Each_Motors.get(0) > Speed_Of_Each_Motors.get(1);
            if(extensial)
            {
               Speed_Of_Each_Motors.set(0,Speed_Of_Each_Motors.get(0)-Half_difference) ;
               Speed_Of_Each_Motors.set(1,Speed_Of_Each_Motors.get(1)+Half_difference) ;

            }
            else
            {
                Speed_Of_Each_Motors.set(0,Speed_Of_Each_Motors.get(0)+Half_difference) ;
               Speed_Of_Each_Motors.set(1,Speed_Of_Each_Motors.get(1)-Half_difference) ;
            }
           }
         }
          //İKİNCİ ALTERNATİF
         // if(Current_Difference > max_Difference)
         // {
         //    if(Speed_Of_Each_Motors.get(0) > Speed_Of_Each_Motors.get(1))
         //    {
         //       double Half_difference = Current_Difference / 2;
         //         Speed_Of_Each_Motors.set(1,Speed_Of_Each_Motors.get(1)+Half_difference);
         //    }
         //    else
         //    {
         //        double Half_difference = Current_Difference / 2;
         //         Speed_Of_Each_Motors.set(0,Speed_Of_Each_Motors.get(0)+Half_difference);
         //    }
         // }
       }

       //Ramping algoritması ile Klavye analogundaki ani hızlanmaları engellediğimiz gibi Joystick'de ani hızlanmaları engelleyip daha pürüzsüz ve akıcı bir hızlanma sağlıyor 
       private double rampMotorInput(double currentSpeed, double joystickInput, double RAMP_RATE) {
         if (Math.abs(joystickInput - currentSpeed) > RAMP_RATE) {
             // Joystick input, mevcut hızdan yeterince farklıysa ramp yap
             if (currentSpeed < joystickInput) {
                 currentSpeed += RAMP_RATE;
             } else if (currentSpeed > joystickInput) {
                 currentSpeed -= RAMP_RATE;
             }
         } else {
             // Joystick input ile mevcut hız arasındaki fark çok küçükse, inputu direkt kullan
             currentSpeed = joystickInput;
         }
         // Hızı -1 ile 1 arasında sınırla
         return Math.max(-1.0, Math.min(1.0, currentSpeed));
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
      
        /***************************/

       /* | Region : MOTOR DURUM İZLEME  |*/


       // !!! PID SİSTEMİ İLE KONTROL YAPILMAKTADIR !!! 
       //Periodic Metot // Hem Teleop Hem de Autonomous için kullanılabilir
        public void Motor_Velocity_Equation(Double Setpoint, Double Current_Speed, Double Current_Direction)
        {
            // Robotun durumunu kontrol et
           if(robot_Status != RobotStatus.IDLE)
           {
            // PID kontrol algoritmasını çalıştır
           //PID'nin coefficient değerlerini PID_parametres fonksiyonundaki değerler belirliyor
           PIDController pid_controller = new PIDController(PID_Coefficients.get(0),PID_Coefficients.get(1), PID_Coefficients.get(2));
           double output = pid_controller.calculate(Setpoint, Current_Speed);

           // Motorların hız kontrol sinyallerini hesapla
           double left_motor_speed = output;
           double right_motor_speed = output;

           // Motorların hız kontrol sinyallerini uygula
           Speed_Of_Each_Motors.set(0, left_motor_speed);
           Speed_Of_Each_Motors.set(1, right_motor_speed);

           // Önceki değerleri kaydet
           double error = Setpoint - Current_Speed;
           previous_error = error;
           integral_sum += error *dt;
           }
          
        }
      public ArrayList<Double> PID_parametres(double k_Proportional, double k_Integral, double k_derivative)
      {
        PID_Coefficients.set(0, k_Proportional) ;
        PID_Coefficients.set(1, k_Integral) ;
        PID_Coefficients.set(2, k_derivative) ;
        return PID_Coefficients;
      }
      public void Robot_Status_Control()
      {
         // ROBOTUN  MOTORLARINDAKİ HIZINA GÖRE MOTORUN BULUNDUĞU HALİ KONTROL EDİYORUZ
         if(Speed_Of_Each_Motors.get(0) < 0.025d && Speed_Of_Each_Motors.get(1) < 0.025d )
         {
            Speed_Of_Each_Motors.set(0, 0d);
            Speed_Of_Each_Motors.set(1, 0d);
           robot_Status = RobotStatus.IDLE;
         }
         else 
         {
             robot_Status = RobotStatus.DYNAMIC;
         }

         //ayrı olarak Robot dinamikse de bir yere gidiyor mu yoksa dönüyor mu bunun kontrolü
         if(Math.abs(Speed_Of_Each_Motors.get(0) - Speed_Of_Each_Motors.get(1)) > 0.3)
         {
          robot_Status = RobotStatus.TURNING;
         }
      }
       /*|Endregion :  MOTOR DURUM İZLEME |*/
 }
      

