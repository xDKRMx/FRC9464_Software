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

public  class MotorControllerModule {
       public enum RobotStatus {
        DYNAMIC,
        IDLE,
        TURNING,
        CONSTANTPOWER
       }
     //Bu class bir nevi bizim için bir çok class içerisinde kullanacağımız ve robottaki bir çok kodun temelini oluşturacak bir class. 
     // Bu class içerisindeki işlemler bir nevi robotun içerisindeki temel dinamikler diyebilirim bu temel dinamiklere örnek olarak sürüş, dönme hareket durma ivmeli hareket vs.
     //Bu class'deki fonksiyonların ve  bir çok değişkeninin public olarak tanımlanması veya OOP(Object oriented) mantığı ile encapsulation olarak tanımlanmış olması kodun ilerleyişi açısından daha iyi olur.
    /************************************************** */ 
    //Ek modüllerin tanımlaması
    KeyboardAnalog Key_Analog = new KeyboardAnalog(this);
    public SensorIntegrationModule Sensor_Integration = new  SensorIntegrationModule(this);
    //Robotun durumu
    public RobotStatus robot_Status;
    // Motorların tanımlamaları
    private  CANSparkMax Left_Leader = new CANSparkMax(0,MotorType.kBrushless);
     private  CANSparkMax Right_Leader = new CANSparkMax(1,MotorType.kBrushless);
      private CANSparkMax Left_Follower= new CANSparkMax(2,MotorType.kBrushless);
     private  CANSparkMax Right_Folower = new CANSparkMax(3,MotorType.kBrushless);
     //Ana differential Drive değişkenimiz
     public DifferentialDrive Main_Robot_Drive = new DifferentialDrive(Left_Leader, Right_Leader);
     //Motora ait bazı verilerin Tanımlamaları
     //iki elemanlı bir ARRAYLIST ilk elemen LEFT MOTOR POWER ikinci eleman RIGHT MOTOR POWER
     /* POWER : MOTORA INPUT İLE VERİLEN GÜÇ (MİN -1, MAX +1)  */
     private ArrayList<Double> Power_Of_Each_Motors;
     //Motorun varsayılarak yapılmış max hızı
     public Double Motor_Max_Speed = 10d;
     //Motorun varsayılarak yapılmış min hızı
     public Double Motor_Min_Speed = -10d;
     //Motorların Motor Stabilitesini sağlama Kontrolü
     /*
      Bu değişkenin işlevi şu eğer ki biz motorların sabit hızla gitmesini istiyorsak bunun için PID sistemli mMotor_Velocity_Equation() fonksiyonu kullanılacak. 
      Eğer ki biz motorların hızını sabit olmasını istemiyorsak bunun yerine motorların arasındaki  hız farkına göre bir yumuşatma işlemi uygulayacaksak Motor_Stability() sisteminde işlem yapılacak Motor_Stability() fonksiyonu kullanılacak.
       Biz bu fonksiyonları uygulamazsak bunun halinde motorlar arası  hız farkı çok olabilir bu da robotu çok hızlı döndürerek bozabilir
      */
     public String Motor_Power_Control;
     private ArrayList<Double> PID_Coefficients;
      public boolean Robot_Turning;
     /**********************************/
     // Constructor
     public MotorControllerModule()
     {
        Power_Of_Each_Motors = new ArrayList<>(Collections.nCopies(2, 0.0));
        Left_Leader.set(0);
        Right_Leader.set(0);
        Left_Leader.restoreFactoryDefaults();
        Right_Leader.restoreFactoryDefaults();
        //Robotun durumunu varsayılan olarak IDLE ayarlama
        robot_Status = RobotStatus.IDLE;
        //Başta Joystick üzerinden herhangi ilgili butona basılmadığı (PID sistemini kullanacak) için motorların hız kontrolü için 
        Motor_Power_Control = "Stability";
        //PID controlleri için başlangıç
        PID_Coefficients = new ArrayList<>(Collections.nCopies(3, 0.0));
     }
     /***************************/

      /* | Region : MOTOR VERİELERİ ÇEKME  |*/
      //Sensör entegrasyon ve Telemetri modüllerinde bu tarz fonksiyonlardan yararlanarak robotun verilerini alacağız
      public ArrayList<Double> Get_Power_Of_Each_Motors()
      {
         return  Power_Of_Each_Motors;
      }
      // Algoritmalar öncesinde başka modüllerde bu Motorların değişkenine ihtiyacımız olacağı için bir GET metodu gerekiyor 
     public CANSparkMax GET_SPARKMAX_Motors(int index)
     {
       CANSparkMax[] SPARKMAX_Motors = {Left_Leader,Left_Follower,Right_Leader,Right_Folower};
       return SPARKMAX_Motors[index];

     }
      /* | End Region : MOTOR VERİELERİ ÇEKME  |*/

     /***************************/
     /*|region : CAN MOTOR KONTROL  |*/

     // Periodic Metot // Hem Teleoperation hem de Autonomous için kullanılabilir 
     public void Follow_Periodic()
     {
       Left_Follower.follow(Left_Leader);
       Right_Folower.follow(Right_Leader);
     }

     //Periodic Metot //Teleop metotlar için
      public void Teleop_Drive_Periodic(Joystick leftJoystick, Joystick rightJoystick)
      {
           //Burada bu joysticklerdeki axis'lerin itilme miktarını çektik
           if(leftJoystick != null && rightJoystick != null)
           {
            // Mevcut motorun güçlerini al
             double currentLeftInput = Power_Of_Each_Motors.get(0);
             double currentRightInput = Power_Of_Each_Motors.get(1);
             // Joystick inputlarını al veya varsayılan olarak 0 kabul et
             double leftJoystickInput = leftJoystick.getY();
             double rightJoystickInput =  rightJoystick.getY();
             // Ramp algoritmasına göre Stabiliteyi sağlayacak bir ayarlama işlemi
             Double Absolute_Left_Motor_Power = rampMotorInput(currentLeftInput,leftJoystickInput,0.1f);
             Double Absolute_Right_Motor_Power = rampMotorInput(currentRightInput,rightJoystickInput,0.1f);
            Power_Of_Each_Motors.set(0, Absolute_Left_Motor_Power);
            Power_Of_Each_Motors.set(1,Absolute_Right_Motor_Power );
           }
           else
           {
             //Eğer ki joystickler tanımlı depilse joystick'in görevlerini geçici süreliğine Klavyeden yazılan tuşlar üstlenecek
             Key_Analog.periodic_KeyListener();
             if(robot_Status != RobotStatus.TURNING && robot_Status != RobotStatus.CONSTANTPOWER)
             {
              Power_Of_Each_Motors.set(0,  Key_Analog.Motor_Speed_Key_Analog("Left"));
              Power_Of_Each_Motors.set(1,  Key_Analog.Motor_Speed_Key_Analog("Right"));
             }
             
           }

          //ARCADE DRIVE ALGORİTMASI
          //* */
          //Arcade Drive sürüş tekniği şu şekildedir. Sürücünün robotu yönetmek için elinde tek bir joystick vardır bu Josytick'deki ilk arduinosu robotun hareketini kontrol edebilmemizi sağlar joystick'deki diğer arduinosu ise robotun ileri ve geri hareketini sağlar
          // burada tek bir joystic olduğu için kontrol olarak Tank Drive'dan daha rahattır ancak daha klasik hareketlere olanak tanır
          //Joystick'de  hem sol motoru  hem de sağ motoru aynı motor olarak varsayıp tek bir motor olarak ayarlamalıyız
          //Arcade drive'da bu yüzden ilk paramatre motora verilen hızı sağdaki parametre ise dönüşü sağlar
         //Main_Robot_Drive.arcadeDrive( Power_Of_Each_Motors.get(0),Power_Of_Each_Motors.get(1));

         //TANK DRIVE ALGORİTMASI
         //* */
          // Tank Drive sürüş tekniği şu şekildedir sürücünün robotu yönetmek için kullanacağı iki tane joystic vardır birisi sağ diğeri solu yönetir ve sürücü robotun hareket için bu Josyticklerin itilme miktarına göre robota harektini verir
          // Örneğin robotun ileri gitmesi için iki josytic'de aynı oranda ileri itilmeli sola doğru gitmesi için sağdaki josytick soldakinden daha fazla itilerek sağdaki motorun sola sapması sağlanması
          //* */
          //Robotumuzu tank Drive bir şekilde geliştireceğimiz için bu tanımladığımız Right ve Left Leader'laarın motorlarına verilen bir güç olmalı 
         //Bu gücü de Input Processing Class'i içerisinde tanımlanmış joystick'lerin yön tuşlarındaki kolun ne kadar itildiğine bağlı olarak alacağı değere bağlı olacak. Artı olarak parametrelere değerini Input Processing'deki Joystickler Verecek 

          if(Motor_Power_Control == "Stability")
          {
            if(robot_Status != RobotStatus.TURNING)
            {
             Motor_Stability(0.2d);
            }
          
          }
          else if(Motor_Power_Control == "PID")
          {
            if(robot_Status != RobotStatus.TURNING)
            {
             // PID sistemli motor güç kontrolünü çağırın
              Double[] Current_Speed = Sensor_Integration.Get_Motors_Speed();
             //BURADAKİ DEĞERLER DENEME YANILMA MAKSADIYLA GİRİLMİŞTİR DEĞİŞTİRİLEBİLİR
             // PID_parametres(0.1,0.01,0.001);
             //Buradaki SetPoint değeri İstenilen hızı temsil etmektedir 
             //bizim PID algoritmasını yazarken ki amacımız aslında hem sağ hem de sol motorun hızını istenilen değere yani Setpoint değerine sabitlemek (Set point değeri değişken olabilir ancak bu bir hızı değeridir, motorların istenilen hız değeri)
             Motor_Velocity_Equation((Power_Of_Each_Motors.get(0) + Power_Of_Each_Motors.get(1)) / 2, Current_Speed);
             }
          }
           Main_Robot_Drive.tankDrive( Power_Of_Each_Motors.get(0),Power_Of_Each_Motors.get(1));
       }

       //Ramping algoritması ile Klavye analogundaki ani hızlanmaları engellediğimiz gibi Joystick'de ani hızlanmaları engelleyip daha pürüzsüz ve akıcı bir hızlanma sağlıyor 
        private double rampMotorInput(double currentpower, double joystickInput, double RAMP_RATE) {
         if (Math.abs(joystickInput - currentpower) > RAMP_RATE) {
             // Joystick input, mevcut hızdan yeterince farklıysa ramp yap
             if (currentpower < joystickInput) {
                 currentpower += RAMP_RATE;
             } else if (currentpower > joystickInput) {
                 currentpower -= RAMP_RATE;
             }
         } else {
             // Joystick input ile mevcut hız arasındaki fark çok küçükse, inputu direkt kullan
             currentpower = joystickInput;
         }
         // Hızı -1 ile 1 arasında sınırla
         return Math.max(-1.0, Math.min(1.0, currentpower));
        }

        //Robotun İstenilen yönde dönmesini Sağlayan Turning metodu bu metod ile biz motora istediğimiz gibi saat yönünde veya saat yönünün tersine bir rotasyon işlemi uygulayabileceğiz (ROBOT HEM STATİKKEN HEM DE HAREKET HALİNDEYKEN)
        //Periodic kontrol metodu 
        // Hem Teleop hem de Autonomous için kullanılabilir
        public void Rotate_Robot(double Turning_Speed,Boolean Positive_Rotation)
        {
          //Burada biz robotu döndürme işlemini yaptıkça robotun durumu TURNING'dir ve turning içerisinde robotun bir çok işlemi yapmasına izin verilmez /*ÇAKIŞMA OLMAMASI İÇİN */
              robot_Status = RobotStatus.TURNING;
              Power_Of_Each_Motors.set(0,  Positive_Rotation ? -Turning_Speed : Turning_Speed);
              Power_Of_Each_Motors.set(1,   Positive_Rotation ? Turning_Speed : -Turning_Speed);
        } 
        public void Stop_Rotating()
        {
          robot_Status = RobotStatus.DYNAMIC;
        }

    /*|Endregion : CAN MOTOR KONTROL  |*/
        /***************************/

       /* | Region : MOTOR DURUM İZLEME  |*/

       //Motorlara verilen güçlerin stabilitesi ve max motor gücü farkı
       void Motor_Stability(Double max_Difference)
       {
         double Current_Difference = Math.abs(Power_Of_Each_Motors.get(0) - Power_Of_Each_Motors.get(1));
         robot_Status = RobotStatus.DYNAMIC;
        // Current Difference Control
        //İLK ALTERNATİF
         if(Current_Difference > max_Difference)
         {
            double Extra_Difference = (Current_Difference -max_Difference ) / 2;
            Boolean extensial = Power_Of_Each_Motors.get(0) > Power_Of_Each_Motors.get(1);
            if(extensial)
            {
               Power_Of_Each_Motors.set(0,Power_Of_Each_Motors.get(0)-Extra_Difference) ;
               Power_Of_Each_Motors.set(1,Power_Of_Each_Motors.get(1)+Extra_Difference) ;

            }
            else
            {
                Power_Of_Each_Motors.set(0,Power_Of_Each_Motors.get(0)+Extra_Difference) ;
               Power_Of_Each_Motors.set(1,Power_Of_Each_Motors.get(1)-Extra_Difference) ;
            }
         }
          //İKİNCİ ALTERNATİF
         // if(Current_Difference > max_Difference)
         // {
         //    if(Power_Of_Each_Motors.get(0) > Power_Of_Each_Motors.get(1))
         //    {
         //       double Half_difference = Current_Difference / 2;
         //         Power_Of_Each_Motors.set(1,Power_Of_Each_Motors.get(1)+Half_difference);
         //    }
         //    else
         //    {
         //        double Half_difference = Current_Difference / 2;
         //         Power_Of_Each_Motors.set(0,Power_Of_Each_Motors.get(0)+Half_difference);
         //    }
         // }
       }

       // !!! PID SİSTEMİ İLE KONTROL YAPILMAKTADIR !!! //
       //Periodic Metot // Hem Teleop Hem de Autonomous için kullanılabilir
         void Motor_Velocity_Equation(Double Desired_Speed, Double[] Current_Speed)
        {
            // Robotun durumunu kontrol et
           if(robot_Status != RobotStatus.IDLE)
           {
           // PID kontrol algoritmasını çalıştır
           //PID'nin coefficient değerlerini PID_parametres fonksiyonundaki değerler belirliyor
           /*
            * BU KOD TAM İSTENİLDİĞİ GİBİ ÇALIŞMADIĞI İÇİN YORUM SATIRINA ALINMIŞTIR ŞİMDİLİK İŞLEVİ SADECE İKİ MOTOR ARASINDAKİ GÜÇLERİNİN ORTALAMASINDA MOTORLARIN GÜÇLERİNİ EŞİTLEMEK
            */
          //PIDController pid_controller = new PIDController(PID_Coefficients.get(0),PID_Coefficients.get(1), PID_Coefficients.get(2)); 
          // //Bu çıktı değerleri robotun hızını veriyor ve bu hız değeri 1 ile -1 arasında olmayabilir biz robota hız değeri değil güç değeri vereceğimiz için robotun hızı ne ise ona göre -1 ile 1 arasında bir değere ölçeklendirelim
          // double output_Left = pid_controller.calculate(Desired_Speed, Current_Speed[0]);
          // double output_Right = pid_controller.calculate(Desired_Speed, Current_Speed[1]);
          // // Çıktı değerlerini ölçeklendir (-1 ile 1 arasında )
          // Double Output_Left_Scaled =  output_Left > 0 ? (output_Left - Motor_Min_Speed) / (Motor_Max_Speed - Motor_Min_Speed) : (output_Left - Motor_Max_Speed) / (Motor_Max_Speed - Motor_Min_Speed);
          // Double Output_Right_Scaled =  output_Left > 0 ?(output_Right - Motor_Min_Speed) / (Motor_Max_Speed - Motor_Min_Speed) : (output_Right - Motor_Max_Speed) / (Motor_Max_Speed - Motor_Min_Speed);
          // Motorların hız kontrol sinyallerini hesapla
          // double left_motor_speed = Desired_Speed;
          // double right_motor_speed = Desired_Speed;
           // Motorların hız kontrol sinyallerini uygula
           //ROBOTUN MOTORLARINA SABİT GÜÇ UYGULANARAK HAREKET SAĞLANMASI İÇİN ROBOTA ÖZEL DURUM EKLENMİŞTİR
           robot_Status = RobotStatus.CONSTANTPOWER;
           Power_Of_Each_Motors.set(0, Desired_Speed);
           Power_Of_Each_Motors.set(1, Desired_Speed);
         }
        }
      public ArrayList<Double> PID_parametres(double k_Proportional, double k_Integral, double k_derivative)
      {
        PID_Coefficients.set(0, k_Proportional) ;
        PID_Coefficients.set(1, k_Integral) ;
        PID_Coefficients.set(2, k_derivative) ;
        return PID_Coefficients;
      }
       // ROBOTUN  MOTORLARINDAKİ HIZINA GÖRE MOTORUN BULUNDUĞU HALİ KONTROL EDİYORUZ
      public void Robot_Status_Control()
      {
        if( robot_Status != RobotStatus.TURNING && robot_Status != RobotStatus.CONSTANTPOWER)
        {
           if(Math.abs(Power_Of_Each_Motors.get(0)) < 0.025d && Math.abs(Power_Of_Each_Motors.get(1)) < 0.025d )
          {
             Power_Of_Each_Motors.set(0, 0d);
             Power_Of_Each_Motors.set(1, 0d);
            robot_Status = RobotStatus.IDLE;
          }
          else 
          {
              robot_Status = RobotStatus.DYNAMIC;
          }
        }
      }
       /*|Endregion :  MOTOR DURUM İZLEME |*/
 }
      

