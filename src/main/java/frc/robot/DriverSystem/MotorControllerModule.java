package frc.robot.DriverSystem;


import frc.robot.ManipulationSystem.*;
import frc.robot.ManipulationSystem.ShooterModule.ShooterMotorStatus;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
//Simülasyonu çalıştırmak için Keyboard Analog importu (Geçici)
import frc.robot.DriverSystem.AdditionalClasses.Pose2dSendable;


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
    public SensorIntegrationModule Sensor_Integration = new  SensorIntegrationModule(this);
    public ShooterModule Shooter_Module = new  ShooterModule(this);
    public ClimberModule Climber_Module = new  ClimberModule(this);
    public VisionProcessing VisionProcessing = new  VisionProcessing();
   // KeyboardAnalog Key_Analog = new KeyboardAnalog(this);
    //Robotun durumu
    public RobotStatus robot_Status;
    //Robotun durumunu mekaniksel değil de durumsal yapma işlemi 
    public Boolean Situational = false;
    // Motorların tanımlamaları
    private  CANSparkMax Left_Leader = new CANSparkMax(1,MotorType.kBrushless);
     private  CANSparkMax Right_Leader = new CANSparkMax(2,MotorType.kBrushless);
      private CANSparkMax Left_Follower= new CANSparkMax(3,MotorType.kBrushless);
     private  CANSparkMax Right_Folower = new CANSparkMax(4,MotorType.kBrushless);
     //Ana differential Drive değişkenimiz
     public DifferentialDrive Main_Robot_Drive = new DifferentialDrive(Left_Leader, Right_Leader);
     //Motora ait bazı verilerin Tanımlamaları
     //iki elemanlı bir ARRAYLIST ilk elemen LEFT MOTOR POWER ikinci eleman RIGHT MOTOR POWER
     /* POWER : MOTORA INPUT İLE VERİLEN GÜÇ (MİN -1, MAX +1)  */
     private ArrayList<Double> Motor_Power_List;
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
      public boolean Stop_Rotate;
     //OTONOM KISMI İÇİN TANIMLANMACAK DEĞİŞKENLER
     private Timer timer = new Timer();
     private boolean Is_Point_Setted = false;
     private boolean reached_Setpoint = false;
     private boolean reached_Angle = false;
     private double Initial_Angle_Difference = 999d;
     double TargetAngle;
     double CurrentAngle;
     //ODOMETRİ HESABI İLE
     Pose2d Current_Point;
     Pose2d Setpoint;
     /*GEÇİCİ SÜRELİĞİNE OLUŞTURULMUŞ MANEVRA SİSTEMİ TESTİ */
     ArrayList<Pose2d> OBSTACLES = new ArrayList<>(Collections.nCopies(5, null));
     //Senör verilerine göre
    //  Double[]  Current_Point_Sensor = new Double[2];
    //  Double[] Setpoint_Sensor = new Double[2];
     //Robotun manevra işlemine girmesi için limit değişkenler
     private double Limit_Distance = 2d;
     private double limit_Velocity = 5d;
     private boolean manoeuvre = false;
     /**********************************/
     // Constructor
     public MotorControllerModule()
     {
        Motor_Power_List = new ArrayList<>(Collections.nCopies(2, 0.0));
        Left_Leader.set(0);
        Right_Leader.set(0);
        Left_Leader.restoreFactoryDefaults();
        Right_Leader.restoreFactoryDefaults();
        //Inverted işlemleri
        Left_Leader.setInverted(true);
        Right_Leader.setInverted(false);
        //Robotun durumunu varsayılan olarak IDLE ayarlama
        robot_Status = RobotStatus.IDLE;
        //Başta Joystick üzerinden herhangi ilgili butona basılmadığı (PID sistemini kullanacak) için motorların hız kontrolü için 
        Motor_Power_Control = "Stability";
        //PID controlleri için başlangıç
        PID_Coefficients = new ArrayList<>(Collections.nCopies(3, 0.0));
        //zamanlayıcıyı sıfırla
        timer.reset();
        timer.start();
     }
     /***************************/

     public Boolean CCRP_Check(double limelight_Height , double target_Height){
          
      if(VisionProcessing.hasValidTarget() && (target_Height - limelight_Height) / Math.tan(VisionProcessing.getTargetOffsetY() * (3.14159d/180.0d)) <= 20d){
           return true;
         }
         else return false;
      }
     
     /* | Region : MOTOR VERİELERİ ÇEKME  |*/
      //Sensör entegrasyon ve Telemetri modüllerinde bu tarz fonksiyonlardan yararlanarak robotun verilerini alacağız
      public ArrayList<Double> Get_Motor_Power_List()
      {
         return  Motor_Power_List;
      }
      // Algoritmalar öncesinde başka modüllerde bu Motorların değişkenine ihtiyacımız olacağı için bir GET metodu gerekiyor 
     public CANSparkMax GET_SPARKMAX_Motors(int index)
     {
       CANSparkMax[] SPARKMAX_Motors = {Left_Leader,Left_Follower,Right_Leader,Right_Folower};
       return SPARKMAX_Motors[index];

     }
     public double Get_Timer()
     {
      double Current_time = timer.get();
      return Current_time;
     }

     //Motorlar aracılığıyla robotun birim zaman içerisinde tükettiği toplam enerjiyi hesaplıyoruz
    public double Get_Energy_Consuption() {
        // Geçen süreyi al ve timer'ı sıfırla
        double elapsedTime = Get_Timer();
         //robotun her bir motorun tükkettiği toplam enerjiyi bulmak için fizikteki E = V * I * t bağıntısını kullanıyoruz
         //E : tüketilen enerji, V : robotun motorlarındaki voltaj (gerilimi), I: robotun motorlarından geçen akım, t : birim zaman
        // Motorların anlık akımını 
        double leftMotorCurrent = Left_Leader.getOutputCurrent();
        double rightMotorCurrent = Right_Leader.getOutputCurrent();
        double leftMotorVoltage = Left_Leader.getBusVoltage();
        double rightMotorVoltage = Right_Leader.getBusVoltage();
        
        // Her iki motor için enerji tüketimini hesapla ve topla
        double leftMotorEnergy = leftMotorVoltage * leftMotorCurrent * elapsedTime / 3600.0; // Wh cinsinden
        double rightMotorEnergy = rightMotorVoltage * rightMotorCurrent * elapsedTime / 3600.0; // Wh cinsinden
        // Toplam enerji tüketim ortalamasını SmartDashboard üzerinde göster
        double Total_Energy_Consumption = leftMotorEnergy + rightMotorEnergy;
        return Total_Energy_Consumption;
    }

      /* | End Region : MOTOR VERİLERİ ÇEKME  |*/

    
     /***************************/
     /*|region : MOTOR VERİLERİ SET ETME  |*/
     public void Set_Power(Double Motor_Power)
     {
      Motor_Power_List.set(0, Motor_Power);
     }
     /*|End region : MOTOR VERİLERİ SET ETME  |*/

     /*|region : CAN MOTOR KONTROL  |*/

   
     /*|Title : TELOP MOTOR KISMI  |*/
     // Periodic Metot // Hem Teleoperation hem de Autonomous için kullanılabilir 
     public void Follow_Periodic()
     {
       Left_Follower.follow(Left_Leader);
       Right_Folower.follow(Right_Leader);
     }

     //Periodic Metot //Teleop metotlar için
      public void Teleop_Drive_Periodic(Joystick joystick)
      {
           //Burada bu joysticklerdeki axis'lerin itilme miktarını çektik
           if(joystick != null)
           {
             if(robot_Status != RobotStatus.TURNING && robot_Status != RobotStatus.CONSTANTPOWER)
             {
                 // Mevcut motorun güçlerini al
               double currentLeftInput = Motor_Power_List.get(0);
               double currentRightInput = Motor_Power_List.get(1);
               // Joystick inputlarını al veya varsayılan olarak 0 kabul et
               double Speed_Input = Math.pow(joystick.getRawAxis(1),3);
               double Rotate_Input = Math.pow(joystick.getRawAxis(2),5);
               //Eşik değer kontrolü
               if(Math.abs(Speed_Input) < 0.05)Speed_Input = 0;
               if(Math.abs(Rotate_Input) < 0.05)Rotate_Input = 0;
              //  // Ramp algoritmasına göre Stabiliteyi sağlayacak bir ayarlama işlemi
               Double Absolute_Left_Motor_Power = rampMotorInput(currentLeftInput,Speed_Input,0.1f);
               Double Absolute_Right_Motor_Power = rampMotorInput(currentRightInput,Rotate_Input,0.1f);
                Motor_Power_List.set(0, Absolute_Left_Motor_Power);
                Motor_Power_List.set(1,Absolute_Right_Motor_Power );
             }
           }
           else
           {
            /*ROBOTUN DEPLOY EDERKEN DX11 hatası vermemesi için keyboard ile alakalı olan kodlar yorum satırına alınd */
            /****** */
             //Eğer ki joystickler tanımlı depilse joystick'in görevlerini geçici süreliğine Klavyeden yazılan tuşlar üstlenecek
            //  Key_Analog.periodic_KeyListener();
            //  if(robot_Status != RobotStatus.TURNING && robot_Status != RobotStatus.CONSTANTPOWER)
            //  {
            //   Motor_Power_List.set(0,  Key_Analog.Motor_Speed_Key_Analog("Left"));
            //   Motor_Power_List.set(1,  Key_Analog.Motor_Speed_Key_Analog("Right"));
            //  }
             
           }

          //ARCADE DRIVE ALGORİTMASI
          //* */
          //Arcade Drive sürüş tekniği şu şekildedir. Sürücünün robotu yönetmek için elinde tek bir joystick vardır bu Josytick'deki ilk arduinosu robotun hareketini kontrol edebilmemizi sağlar joystick'deki diğer arduinosu ise robotun ileri ve geri hareketini sağlar
          // burada tek bir joystic olduğu için kontrol olarak Tank Drive'dan daha rahattır ancak daha klasik hareketlere olanak tanır
          //Joystick'de  hem sol motoru  hem de sağ motoru aynı motor olarak varsayıp tek bir motor olarak ayarlamalıyız
          //Arcade drive'da bu yüzden ilk paramatre motora verilen hızı sağdaki parametre ise dönüşü sağlar
         //Main_Robot_Drive.arcadeDrive( Motor_Power_List.get(0),Motor_Power_List.get(1));

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
             // Motor_Stability(0.2d);
            }
          }
          else if(Motor_Power_Control == "Constant")
          {
            if(robot_Status != RobotStatus.TURNING)
            {
             // PID sistemli motor güç kontrolünü çağırın
              Double[] Current_Speed = Sensor_Integration.Get_Motors_Speed();
             //BURADAKİ DEĞERLER DENEME YANILMA MAKSADIYLA GİRİLMİŞTİR DEĞİŞTİRİLEBİLİR
             // PID_parametres(0.1,0.01,0.001);
             //Buradaki SetPoint değeri İstenilen hızı temsil etmektedir 
             //bizim PID algoritmasını yazarken ki amacımız aslında hem sağ hem de sol motorun hızını istenilen değere yani Setpoint değerine sabitlemek (Set point değeri değişken olabilir ancak bu bir hızı değeridir, motorların istenilen hız değeri)
             Motor_Velocity_Equation((Motor_Power_List.get(0)), Current_Speed);
             }
          }
          //System.out.println(Motor_Power_Control + " " + robot_Status);
            Main_Robot_Drive.arcadeDrive( Motor_Power_List.get(0),Motor_Power_List.get(1),false); 
         //  System.out.println(Left_Leader.get() + " Left motor " + Right_Leader.get() + " right ");
         double Deneme = VisionProcessing.Scan_Apriltag();
         System.out.println(Deneme + " ID");
         }
       /*|END Title : TELOP MOTOR KISMI  |*/

       /*| Title : OTONOM MOTOR KISMI  |*/

       //Bu kısımda robotun motor kontrol modülündeki algoritmalara göre otonom kısımda yapacağı işlemler vardır
       // Init metot // Otonom kısım için
       public void Autonomous_Drive_Init()
       {
        //burada timer'ı resetleyip başlatıyoruz bu sayede robotun otonom kısım içerisinde yapacağı işlemlerin saniye cinsinden sürelerini kontrol edebiliriz
         timer.reset();
         timer.restart();
       }
     
       public void Autonomous_Drive_Periodic() {
        /*OTONOMDA YAPILACAK İŞLEMLER */
         // ilgili noktaya dönme, döndükten sonra hopörlöre atış işlemi ardından tekrar eski başlangıç açısına dönme
           //eski başlangıç açısına yakın bir değere dönüp taksi yapma taksi yapılıp belli bir mesafe katedildikten sonra kendi ekseninde dönüp ilgili kaynak noktasının Apriltagini algılma 
           //Kaynak noktasının apriltag'i algılandıktan sonra limelight dökümentasyonundaki mesafe ölçümü ile uzaklık bulunduktan sonra o uzaklğının birazcık daha az değeri ile robota gitmesi için bir setpoint noktası belirleyip oraya yöneltme
           //Robotun her frame otonomdaki işlemleri matematiksel olarak değerlendirip onu koordinat sistemine göre değerlendireceğiz
            //Robotun varolan açısını bulma
            /* CurrentAngle =Sensor_Integration.Get_Rotation_Angle(); */
            Current_Point = TelemetryModule.Pose_Sendable.GetPose(); 
            CurrentAngle =Sensor_Integration.Get_Rotation_Angle();
          //  CurrentAngle %= 360;
          //  if(CurrentAngle < 0) CurrentAngle += 360;
          //  CurrentAngle =  CurrentAngle /180;
          
           
           if(timer.get() < 2)
           {
            //İlk 2 saniye boyunca robot kendini notayı atacak şekilde ayarladıktan sonra robotun içindeki notayı hopörlöre atma işlemi
            //Notanın robot içinde olup olmaması Touch sensörü ile kontrol ediliyor
             boolean Note_In_Robot = Sensor_Integration.Note_Touch_Control();
             if(Note_In_Robot) Shooter_Module.Shoot_Subsystem("Speaker","Shooter");
             else Shooter_Module.SlowDown_Motor_Power("Shooter");
           }
           else if (timer.get() >= 2 && timer.get() <= 4) {
               if(Shooter_Module.Shooter_Status == ShooterMotorStatus.Dynamic) Shooter_Module.SlowDown_Motor_Power("Shooter");
               if(reached_Angle == false)
               {
               //Robotun varolan açısına geri dönmesi 
               if( Initial_Angle_Difference==999d)
                {
                  System.out.println("Deneme");
                  TargetAngle =CurrentAngle +  1;
                  if(TargetAngle >2 ) TargetAngle -= 2;
                   if(TargetAngle < 0 ) TargetAngle += 2;
                  Initial_Angle_Difference  =  Math.abs(CurrentAngle - TargetAngle);
                }
                System.out.println((TargetAngle - CurrentAngle)* 180 + " Difference");
               if(!Stop_Rotate)
               {
                if(Math.abs(CurrentAngle - TargetAngle)*180>  1d )
                {
                    double Turning_Speed =  (Math.abs(TargetAngle - CurrentAngle ) / Initial_Angle_Difference ) ;
                    Rotate_Robot(Turning_Speed , (TargetAngle - CurrentAngle) > 0 ? true : false );
                }
               }
                if(Math.abs(CurrentAngle - TargetAngle)*180 < 1d )
                {
                      Initial_Angle_Difference = 999d;
                      Stop_Rotating();
                      reached_Angle = true;
                      //*deploy */
                    //  Motor_Power_List.set(0, 0d);
                      /* */
                      Motor_Power_List.set(1, 0d);
                }
               }
           }
           else if(timer.get() >=4 && timer.get() <4.4)
           {
             //Robotun taksi yapma işlemi
             Motor_Power_List.set(0,0.3);
             /*Deployda silinecek */
             Motor_Power_List.set(1,0.3);
             //Bir sonraki işlem için ön hazırlık
             Stop_Rotate = false;
           } 
           else if(timer.get() >= 4.4 && timer.get() <= 15)
           {
               //Robot dönüp notayı attıktan sonra eski açısına dönme işlemi
                if(!Stop_Rotate)
                {
                     Double Apriltag_ID = VisionProcessing.Scan_Apriltag();
                     Boolean ID_perceived = Apriltag_ID == 2 || Apriltag_ID == 5 ? true : false;
                     if(!ID_perceived ) Rotate_Robot(0.3 , (TargetAngle - CurrentAngle) > 0 ? true : false );
                     else
                     {
                        if( Initial_Angle_Difference==999d)
                      {
                        TargetAngle = CurrentAngle;
                        Initial_Angle_Difference  =  Math.abs(CurrentAngle - TargetAngle);
                      }
                      System.out.println(TargetAngle  + " Target " + CurrentAngle + " Current");
                     if(!Stop_Rotate)
                     {
                      if(Math.toDegrees(Math.abs(CurrentAngle - TargetAngle))> 0.3d )
                      {
                          double Turning_Speed =  (Math.abs(TargetAngle - CurrentAngle ) / Initial_Angle_Difference ) ;
                          Rotate_Robot(Turning_Speed , (TargetAngle - CurrentAngle) > 0 ? true : false );
                      }
                     }
                      if(Math.toDegrees(Math.abs(CurrentAngle - TargetAngle)) < 0.3d )
                      {
                            Initial_Angle_Difference = 999d;
                            Stop_Rotating();
                            reached_Angle = true;
                            //*deploy */
                            Motor_Power_List.set(0, 0d);
                            /* */
                            Motor_Power_List.set(1, 0d);
                      }
                  }
                 }
                else
                {
                  //Eğer ki stop rotation işlemi true olmuşsa demekki Limelight tarafından ilgili APriltag algılanmış ve kaynak noktasu bulunmuştur 
                  //Buradaki işlemle de robotun April tag ile arasındaki mesafe çekilip bu mesafeye bir tık daha yakın bir noktada ve robotun baktığı doğrultudan bir setpoint oluşturup oraya gitme
                  Double Distance = VisionProcessing.Apriltag_Get_Distance(0d, 0d, 0d, 0.2d);
                 Double Current_X_Position = Math.cos(CurrentAngle) * (Distance - 0.5d);
                 Double Current_Y_Position = Math.sin(CurrentAngle) * (Distance - 0.5d);
                  if(Is_Point_Setted == false)
                  {
                   Setpoint = new Pose2d(Current_X_Position,Current_Y_Position,new Rotation2d(Math.toRadians(TargetAngle)));
                   Is_Point_Setted = true;
                   }
                  TargetAngle = CalculateAngle_Pose2D(Current_Point,Setpoint);
                   if( Initial_Angle_Difference==999d)
                   {
                     Initial_Angle_Difference  =  Math.abs(CurrentAngle - TargetAngle);
                   }
                   double Turning_Speed = ( Initial_Angle_Difference < -90d || Initial_Angle_Difference > 90d ) ? (Math.abs(CurrentAngle - TargetAngle ) / Initial_Angle_Difference )*3  : (Math.abs(CurrentAngle - TargetAngle) / Initial_Angle_Difference ) *2;
                   if(Math.abs(CurrentAngle - TargetAngle ) > 75) Turning_Speed = 1;
                  if(Math.abs(CurrentAngle - TargetAngle) > 3d && !reached_Setpoint  && !manoeuvre && !reached_Angle)
                  {
                    //aradaki açı değeri neredeyse aynı olana kadar robotu en kısa yoldan döndür
                   Rotate_Robot(Turning_Speed , (CurrentAngle - TargetAngle) > 0 ? false : true );
                  }
                  else
                  {
                    //koşul sağlandığında robotu döndürmeyi bırak ve robotun motorlarına bu sefer istenilen Setpoint noktasına gitmesi için güç ver  
                    // Stop_Rotating();
                    if(reached_Angle == false) reached_Angle = true;
                  }
                  if(reached_Angle)
                  {
                   Autonomous_Set_Robot(); //Robotun motorlarına  güç ver 
                   //Robotun hem dönmek için hem de hedeflenen noktaya gitmek için vakit kaybetmemesi için biz robota aynı anda iki tane işlem yaptırıyoruz bu sayede vakit kazanıyoruz.
                   //buradaki işlemde Autonomous_Set_Robot robotun motorlarına ileri gidecek şekilde (Lineer Interpolasyon ile) güç veriyor alttaki sorgulama ise robotun hedef noktası ile  arasında bir açı farkı varsa robotun oraya giderken o noktaya tam gidebilmesi için robotu döndürür.
                   // robot setpoint noktasına giderken iki işlem gerçekleşeceği için optimizasyonda sıkıntı çıkabilir duruma göre kod optimize edilecektir
                   if(Math.abs(CurrentAngle - TargetAngle) > 0.01d && !manoeuvre)
                   {
                    Motor_Power_List.set(1,  (CurrentAngle - TargetAngle) > 0 ? Motor_Power_List.get(1) - Turning_Speed : Motor_Power_List.get(1) + Turning_Speed);
                   }
                  }
                }
           }
           else {
               // Rastgele setpoint belirle // Şu anlık görüntü işleme algoritması yazılmadığından robotun gitmesi için rastgele bir set point belirliyoruz ve robotun motor hızını PID ile kontrol ediyoruz
               
                /*ROBOTUN SENSÖRLERİ ENTEGRE EDİLİNCE PROJEYE VERİLER SANAL POSE2D SINIFI ÜZERİNDEN DEĞİL DİREKT ROBOTUN SENSÖRÜNDEN ÇEKİLECEK */
                /*SENSÖR VERİ */
                // Current_Point = Sensor_Integration.Robot_Current_Position();
                Double Current_X_Position = Current_Point.getX();
                Double Current_Y_Position = Current_Point.getY();
               // sürekli yeni bir Setpoint noktası oluşturmaması için bir kereliğine mahsus robotun en az 5 en fazla 10 metre ilerisinde vye gerisinde olacak şekilde hem ,X hem Y ekseninde, bir random setpoint oluşturuyoruz
               if(Is_Point_Setted == false)
               {
                  Setpoint = SetRandomPoint_Pose2D(Current_X_Position, Current_Y_Position);
                 Is_Point_Setted = true;
               }
               //robotun normal bulunduğu konum ile setpoint noktası arasındaki doğrunun eğimine göre (tana) target angle değerini hesaplıyoruz
               TargetAngle = CalculateAngle_Pose2D(Current_Point,Setpoint);

               //Robotun şu an Gyro benzeri bir sensörler entegre edilmediği yönünü çekmekte zorlanıyoruz sanal ortamda
               //alternatif olarak da Pose2D'de kullanılan rotation2D kullanıyoruz
               //Rotation2D ve Pose2D maç sırasında bizim hesaplayabileceğimiz veri yapılarına sahip değil biz bunları sadece simülasyon sırasında test etmek için kullanabiliriz
               //gerçek robotta testleri koordinat sistemini sensörlerden Encoder sensörünü kullanarak 
               //Yön olarak NavX-MXP sensörünü kullanarak
               //random Setpoint noktası oluşturmak yerine de Limelight 3 ile görüntü işleme sistemi sayesinde en yakındaki AMP veya Hopörlörü bir setpoint noktası olarak atayacağız
               //robotun eğer ki sahip olduğu bir Intake sistemi varsa yerdeki notayı setpoint olarak algılayacak
               //Ardından da buradaki kendi konumu ile gideceği setpoint arasındaki mesafeyi ve buna bağlı olarak error mesafesini UltraSonic sensörle sağlayacak

               CurrentAngle =Current_Point.getRotation().getDegrees();
               //robotun o andaki mevcut açısı  -180 180 derece dışındaysa esas ölçüsü alınır
               CurrentAngle %= 360;
               if (CurrentAngle < 0) CurrentAngle += 360;
               
               //Başlangıçta sahip olduğu yön yani açı değeri ile setpoint ile Current point yani robotun konumu arasındaki açının farkı hesaplanır
               if( Initial_Angle_Difference==999d)
               {
                 Initial_Angle_Difference  =  Math.abs(CurrentAngle - TargetAngle);
               }
               //Lineer interpolasyon sistemi ile robotun yönünü target angle'a yaklaştırmak maksadıyla bir dönme hızı veriliyor.
               //Lineer İnterpolasyon : İKİ sayı arasındaki farkı yine farklı iki sayı arasındaki değere indirgemek. Mesela 40 - 0 arasındaki sayıları 1 ile 0'a indirmek gibi 40 ise sayı değeri 1,  0 ise  0 , 20 ise 0.5 gibi. 
               // Not :  *5 katsayısı daha hızlı dönüp vakit kaybetmemesi için verilmiştir
               double Turning_Speed = ( Initial_Angle_Difference < -90d || Initial_Angle_Difference > 90d ) ? (Math.abs(CurrentAngle - TargetAngle ) / Initial_Angle_Difference )*3  : (Math.abs(CurrentAngle - TargetAngle) / Initial_Angle_Difference ) *2;
                if(Math.abs(CurrentAngle - TargetAngle ) > 75) Turning_Speed = 1;

               //Turning Speed Değeri geldikte sonra robotun baktığı açı ile istenilen açı arasındaki fark 2dereceden fazl olduğu sürece döndürülsün 
               //2 derece olmasının sebebi robotun tam açı değerini bulmak için fazla vakit kaybetmemesi ve bir an önce istenilen yola gitmesi
              if(Math.abs(CurrentAngle - TargetAngle) > 3d && !reached_Setpoint  && !manoeuvre && !reached_Angle)
              {
                //aradaki açı değeri neredeyse aynı olana kadar robotu en kısa yoldan döndür
               Rotate_Robot(Turning_Speed , (CurrentAngle - TargetAngle) > 0 ? false : true );
              }
              else
              {
                //koşul sağlandığında robotu döndürmeyi bırak ve robotun motorlarına bu sefer istenilen Setpoint noktasına gitmesi için güç ver  
                // Stop_Rotating();
                if(reached_Angle == false) reached_Angle = true;
              }
              if(reached_Angle)
              {
               Autonomous_Set_Robot(); //Robotun motorlarına  güç ver 
               //Robotun hem dönmek için hem de hedeflenen noktaya gitmek için vakit kaybetmemesi için biz robota aynı anda iki tane işlem yaptırıyoruz bu sayede vakit kazanıyoruz.
               //buradaki işlemde Autonomous_Set_Robot robotun motorlarına ileri gidecek şekilde (Lineer Interpolasyon ile) güç veriyor alttaki sorgulama ise robotun hedef noktası ile  arasında bir açı farkı varsa robotun oraya giderken o noktaya tam gidebilmesi için robotu döndürür.
               // robot setpoint noktasına giderken iki işlem gerçekleşeceği için optimizasyonda sıkıntı çıkabilir duruma göre kod optimize edilecektir
               if(Math.abs(CurrentAngle - TargetAngle) > 0.01d && !manoeuvre)
               {
                Motor_Power_List.set(1,  (CurrentAngle - TargetAngle) > 0 ? Motor_Power_List.get(1) - Turning_Speed : Motor_Power_List.get(1) + Turning_Speed);
               }
              }
               Pose2dSendable.field2.setRobotPose(Setpoint);
           }
           Main_Robot_Drive.arcadeDrive( Motor_Power_List.get(0),Motor_Power_List.get(1),false); 
       }
        private Pose2d SetRandomPoint_Pose2D(double X_position, double Y_Position)
        {
          //Random Setpoint noktaıs oluşturma algoritması 
              Random Random = new Random();
              boolean chooseNegativity = Random.nextBoolean();
              double Set_Point_X_Position =chooseNegativity ? X_position + 5 + (10 - 5) * Random.nextDouble() : X_position - 5 - (10 - 5) * Random.nextDouble(); // 5 ile 10 arasında rastgele bir çift sayı ekler
              chooseNegativity = Random.nextBoolean();
              double Set_Point_Y_Position = chooseNegativity ? Y_Position + 5 + (10 - 5) * Random.nextDouble() : Y_Position - 5 - (10 - 5) * Random.nextDouble();
              /**/
              //Oluşturlan değerler sonucundaki nokta ile robotun varolan noktası arasındaki geometrik hesaplarla oluşturulmuş eğim yani açı değeri
                double targetAngle = Math.toDegrees(Math.atan2(Set_Point_Y_Position - Y_Position,
                Set_Point_X_Position - X_position));
               /**/
               // o açı değerini radyan cinsine çevirip Pose2D class'ine yollama
               Rotation2d Default_Rotation = new Rotation2d(Math.toRadians(targetAngle));
               Pose2d Setpoint = new Pose2d(Set_Point_X_Position , Set_Point_Y_Position, Default_Rotation);
              for (int a = 0; a < 5 ; a++) {
                Double ex = (5) * Random.nextDouble();
                Pose2d pose = new Pose2d(Set_Point_X_Position -ex ,Set_Point_Y_Position -ex , Default_Rotation );
                OBSTACLES.set(a, pose);
              }
             return Setpoint;
        }

        public double CalculateAngle_Pose2D(Pose2d CurrentPoint, Pose2d SetPoint )
        { 
          //burada iki nokta arasındaki açı farkını trigonometri ile hesaplıyoruz
          // Setpoint'e göre hedef açıyı hesapla
          double targetAngle = Math.toDegrees(Math.atan2(SetPoint.getY() - CurrentPoint.getY(),
          SetPoint.getX() - CurrentPoint.getX()));
          targetAngle %= 360;
          if (targetAngle < 0) targetAngle += 360;
          return targetAngle;
        }
       //Otonom Motor Drive Kodu
       //Başlangıçdaki erro yani iki nokta arasındaki fark absürt bir değer girilerek bir kereye mahsus ilk fark hesaplanmak istenmiştir
       private double Initial_Error = 999d; 
       private void Autonomous_Set_Robot() {
        //SetPoint ile CurrentPoint arasındaki mesafeyi yani error'ü hesaplamak (Dik üçgen yöntemi ile hesaplanmaktadır)
        double x_distance = Setpoint.getX() - Current_Point.getX();
        double y_distance = Setpoint.getY() - Current_Point.getY();
        double error = Math.sqrt(x_distance * x_distance + y_distance * y_distance);
        
        if(Initial_Error == 999d) Initial_Error = error;
        //Roobtun motorlarına verilecek gücün yine Lineer Interpolasyon ile eror mesafesine bağlı olarak verilmesi
        double Power = error / Initial_Error; 
       // System.out.println( error + " ERROR ");
        if (Math.abs(error) < 1d) { // Hedefe yaklaştığında ve eşik değer geçildiğinde artık direkt durma komutu yazılmış
          // Manevra testi için yorum satırına alınmıştır error'ün koşulu normalde 0.1 altı
            //  Motor_Power_List.set(0, 0d);
            // Motor_Power_List.set(1,0d);
            /************** */
            //robotun motorlara güç vermeyi durdurması yerine yeni bir setpoint oluşturup oraya gitmesini sağlaması ve bu şekilde SANAL ORTAMDA sonsuz loop'a sokulması
            Stop_Rotating();
            Is_Point_Setted = false;
            reached_Setpoint = false;
            reached_Angle = false;
            Initial_Angle_Difference = 999d;
        } 
        else {
          //ROBOTUN HEDEFLENEN NOKTAYA YAKLAŞIRKEN ÖNÜNE ÇIKAN HERHANGİ BİR ENGEL ÇIKMASI HALİNDE MANEVRA KONTOLÜ YAPMASI
          //Ultrasonic sensörü aracılığıyla yapılan bu kontrolde eğer ki robotun önüne limit mesafeden daha kısa mesafede bir obje çıkarsa ve o anda robotun hızı limit hızdan fazlaysa robot kendisini döndürerek bir nevi engelden manevra sistemi ile kaçınacak
          //Eğer ki önüne herhangi bir obje çıkmazsa çıksa bile artık hızı yavaşlamışsa boş yere manevra yapmayacak
          /*MANEVRA SİSTEMİ ŞU ANDA GERÇEK SENSÖRLERLE TEST YAPILAMADIĞI İÇİN SADECE YENİ BİR SETPOİNT NOKTASI OLUŞTURMAYA YARIYOR */
           Robot_manoeuvre();
          //Hedef ile arasındaki error mesafesi hala eşik değer üstündeyse robotun motorlarına power oranında güç ver 
          if(!manoeuvre)
          {
            //Eğer ki robotun manevra yapması gerekecek şekilde önünde bir obje yoksa lineer interpolasyon sistemi ile robotun hedef noktaya gitmesi sağlanıyor
            Motor_Power_List.set(0, Power);
          }
           
        }
       }
     /*| END Title : OTONOM MOTOR KISMI  |*/  
       //Ramping algoritması ile Klavye analogundaki ani hızlanmaları engellediğimiz gibi Joystick'de ani hızlanmaları engelleyip daha pürüzsüz ve akıcı bir hızlanma sağlıyor 
       //Periodic metot
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

        //Robotun istenilen yönde dönmesini Sağlayan Turning metodu bu metod ile biz motora istediğimiz gibi saat yönünde veya saat yönünün tersine bir rotasyon işlemi uygulayabileceğiz (ROBOT HEM STATİKKEN HEM DE HAREKET HALİNDEYKEN)
        //Periodic kontrol metodu 
        // Hem Teleop hem de Autonomous için kullanılabilir
        public void Rotate_Robot(double Turning_Speed,Boolean Positive_Rotation)
        {
             //Burada biz robotu döndürme işlemini yaptıkça robotun durumu TURNING'dir ve turning içerisinde robotun bir çok işlemi yapmasına izin verilmez /*ÇAKIŞMA OLMAMASI İÇİN */
              Robot_Status_Situational(1);
              /*DEPLOYDA YORUM SATIRI */
            // Motor_Power_List.set(0, Positive_Rotation ? -Turning_Speed : Turning_Speed);
              /* */
              Motor_Power_List.set(1, Positive_Rotation ? Turning_Speed : -Turning_Speed);
        } 
        public void Stop_Rotating()
        {
          Stop_Rotate = true;
          robot_Status = RobotStatus.DYNAMIC;
        }
        
      /*|Title : ROBOT MANEVRA SİSTEMİ|*/  
      //Periodic metot
      //Otonom Kısmı için

      //Robotun harkeet doğrultusundaki çarpabileceği nokta
      //Bu nokta gerçek dünyada bir yeri temsil etmiyor sadece matematiksel olarak daha kesin ve sağlam bir manevra sistemi oluşturulması için POSE2D kullanılarak sanal bir nokta oluşturulmuştur.
      Pose2d Obstacle_Point;
      //UltraSonic'den gelen mesafe değeri
      double Current_Distance = 0d;
      double Initial_Distance = 999d;
      //Robotun POSE2D'de bulunduğu nokta ile obstacle noktası arasındaki fark
      double Current_Difference_Between_Points = 0d;
      //Obstacle noktasının X Y değerlerinin robotun X Y değerlerine farkı
      Double X_difference = 0d;
      Double Y_difference = 0d;
      double Angle_Difference = 0d;
      public void Robot_manoeuvre()
      {
         /*SENSÖR VERİ */
          //|MANEVRA 1. KISIM GERÇEK ROBOT ÜSTÜNDE SENSÖR VERİLERİNİ SANAL ORTAMA AKTARARAK MANEVRA SİSTEMİNİ OLUŞTURMA|//
          //Robotun çarpabileceği en yakındaki objenin robota uzaklığı ve o andaki hızı ölçülüyor
          //  Current_Distance = Sensor_Integration.Robot_Get_Distance();
          //  Double[] Current_Velocities = Sensor_Integration.Get_Motors_Speed();
          //  // hem sağ hem de sol motorun anlık hızlarının ortalaması alınarak o anki hızın limit hızdan küçük veya büyük oldupu karşılaştırılıyor
          //  Double Current_Velocity = (Current_Velocities[0] + Current_Velocities[1]) / 2;
          //  //Eğer ki Obstacle Noktası mevcut ise robotun POSE2D kullanılarak o noktaya olan uzaklığı hesaplanır ve özellikle robot manevra yaparken beli bir miktar döndükten sonra ne kadar ileri gitmesinde belirleyici olur.
          //  /*ODOMETRİ SİSTEMİ İLE obstacle noktası ile robotun varolan uzaklığının kontrolü */
          //  if(Obstacle_Point != null)
          //  {
          //    X_difference = Math.abs(Current_Point.getX() -Obstacle_Point.getX());
          //    Y_difference = Math.abs(Current_Point.getY() -Obstacle_Point.getY());
          //    Current_Difference_Between_Points = Math.sqrt(X_difference * X_difference + Y_difference * Y_difference);
          //  }
          //  if(Current_Distance != 0d &&  Current_Distance < Limit_Distance &&  Current_Velocity > limit_Velocity )
          //   {
          //        // Bu kısımda eğer ki robotun Ultrasonic'in döndürdüğü veriye göre en yakınındaki objeye olan uzaklığı limit mesafeden küçükse burada yapılacak manevra işlemini daha gerçekçi ve hatasız verilerle yapmak için POSE2D'den yararlanıyoruz
          //        //robotun manevra yapmadan önceki son distance mesafesini robotun koordinat sistemi üzerinden bulunduğu konuma ekleyerek robotun çarpacağı objeyi Obstacle noktası olarak işaretleme
          //        if(Initial_Distance == 999d)
          //        {
          //           Initial_Distance =Current_Distance;
          //           Obstacle_Point = Set_ObstaclePoint(Initial_Distance);
          //        }
          //        // Eğer ki limit meesafeden daha az bir mesafe kalmışsa ve robotun hızı da limit hızdan fazla ise robotun hızını lineer interpolasyon ile azaltıp bir eşik değerinden sonraki değerlede robotu döndrüyoruz 
          //        double Power = Current_Distance / Limit_Distance; 
          //         if (Math.abs(Power) < 0.5d) { // Hedefe yaklaştığında ve eşik değer geçildiğinde artık direkt robotun manevra yapabilmesi için robotu döndürüyoruz
          //            manoeuvre = true;
          //         } else {
          //           //Hedef ile arasındaki error mesafesi hala robotun manevra yani dönme işlemini yapacak değerin üstündeyse robota manevra işlemine girene kadar motora verilen gücü yavaş yavaş azalt 
          //             Motor_Power_List.set(0, Power);
          //         }
          //   }
          //   else
          //   {
          //    //eğer ki robotun Robotun çarpabileceği en yakındaki objenin robota uzaklığı limit uzaklıktan küçük olup aynı zamandan da robotun mevcut hızının  limit hızdan daha büyük değilse rotate ile manevra işlemi yapma
          //    Stop_Rotating();
          //   }
          //   //UltraSonic sensörden alınan verilerde istenilen koşul sağlanırsa buradaki manevra işlemi gerçekleşecek
          //    if(manoeuvre)
          //    {
          //      //Manevra sisteminde robotun durumuna göre 3 farklı durum söz konusudur
          //       if(Current_Distance < Limit_Distance)
          //      {
          //        //Birinci durum olan bu koşul robotun hem limit mesafeden daha kısa bir mesafede ve baktığı açının da obstacle noktasına doğru olduğunu gösterir
          //        //bu durumdayken robotun dönmesi sağlanarak o noktaya çarpması engellenir
          //        Rotate_Robot(0.5d,true); 
          //      }
          //      else if(Current_Distance > Limit_Distance && Current_Difference_Between_Points < Limit_Distance)
          //      {
          //        //İkinci durumda robotun bulunduğu nokta yine obstacle noktasının limit mesafesinden daha kısa ancak bu sefer robotun hareket doğrultusunda değil ise robotun bu sefer o noktadan kaçınmak için baktığı noktaya doğru ilerlemesi sağlanır
          //         Motor_Power_List.set(0, 0.5d);
          //      }
          //      else
          //      {
          //        //üçüncü durumda robot eğer ki iki durumu da sağlıyorsa manevra işlemi bitirilerek tekrar gitmesi gereken noktaya gitmesi sağlanır
          //        Stop_Rotating();
          //        manoeuvre = false;
          //        reached_Angle = false;
          //        Initial_Angle_Difference = 999d;
          //        Initial_Distance = 999d;
          //        Obstacle_Point = null;
          //      }
          //    }

          
         //|MANEVRA 2. KISIM SANAL ORTAMDA TEST İÇİN YAZILMIŞ SANAL POSE2D DEĞERLERİNE GÖRE MANEVRA SİSTEMİ|//
         /*******************/
        //SENSÖR KULLANMADAN DİJİTAL ORTAMDA TEST KODU
        //BU KODUN ALGORİTMASI ROBOTUN SETPOİNT BELLİ BİR LİMİTTE YAKLAŞTIĞI ZAMAN O NOKTADA MOTORLARA GÜCÜ AZALTIP MANEVRA YAPMASI VE KENDİSİNE YENİ BİR SET POİNT OLUŞTURUP BU SEFER ORAY GİTMESİNİ SAĞLAMAK
        //robotun manevra hareketi öncesi başlangıç açısını çekmek
        //GEÇİCİ MANEVRA SİSTEMİ KONTROLÜ İÇİN OLUŞTURULMUŞ SANAL OBSTACLE NOKTALARI
        //BU TEST KODUDUR GERÇEK ROBOTTA İŞLEVİ YOKTUR
        /* */
        if(Obstacle_Point != null)
         {
          //Eğer ki obstacle noktası oluşturulmuş ise bu obatacle noktasının POSE2D kullanılarak Robota olan uzaklığı ve Robot ile Obstacle noktasının sahip olduğu açı farkı
            X_difference = Math.abs(Current_Point.getX() -Obstacle_Point.getX());
            Y_difference = Math.abs(Current_Point.getY() -Obstacle_Point.getY());
            Angle_Difference = Math.abs(Current_Point.getRotation().getDegrees() -  Obstacle_Point.getRotation().getDegrees());
            Current_Distance = Math.sqrt(X_difference * X_difference + Y_difference * Y_difference); 
         }
         //Obstacle noktası ile robotun bulunduğu konumun Pose2D karşılaştırması sonucu değerler limit değerlere uygun değilse manevra sistemine geçilsin
         if( Current_Distance  != 0d && Current_Distance < Limit_Distance &&  Angle_Difference < 20) manoeuvre = true;
         //UltraSonic sensörden alınan verilerde istenilen koşul sağlanırsa buradaki manevra işlemi gerçekleşecek
         if(manoeuvre)
         {
          System.out.println(Current_Distance + " deneme");
           //Manevra sisteminde robotun durumuna göre 3 farklı durum söz konusudur
            if(Current_Distance < Limit_Distance && Angle_Difference < 20)
           {
             //Birinci durum olan bu koşul robotun hem limit mesafeden daha kısa bir mesafede ve baktığı açının da obstacle noktasına doğru olduğunu gösterir
             //bu durumdayken robotun dönmesi sağlanarak o noktaya çarpması engellenir
             Rotate_Robot(0.5d,true); 
           }
           else if(Current_Distance < Limit_Distance && Angle_Difference > 20)
           {
             //İkinci durumda robotun bulunduğu nokta yine obstacle noktasının limit mesafesinden daha kısa ancak bu sefer robotun hareket doğrultusunda değil ise robotun bu sefer o noktadan kaçınmak için baktığı noktaya doğru ilerlemesi sağlanır
              Motor_Power_List.set(0, 0.5d);
           }
           else
           {
             //üçüncü durumda robot eğer ki iki durumu da sağlıyorsa manevra işlemi bitirilerek tekrar gitmesi gereken noktaya gitmesi sağlanır
             Stop_Rotating();
             manoeuvre = false;
             reached_Angle = false;
             Initial_Angle_Difference = 999d;
             Obstacle_Point = null;
           }
         }
         else
         {
            for (Pose2d pose2d : OBSTACLES) {
            //Buradaki Foreach döngüsü Sanal ortamda oluşturulmuş olan obstacle noktalarının sıra sıra obstacle noktası olması sağlanır
            Obstacle_Point = pose2d;
            //manevra sistemi çalışmıyorsa gösterge olarak green ghost robot modelini obstacle noktasında göster
            Pose2dSendable.field3.setRobotPose(Obstacle_Point);
          }
         }
         
         /****************** */
    }
    //Bu kod bizim manevra işlemi sırasında daha kesin ve sağlam sonuçlar vermemiz için robotun çarpabileceği noktayı POSE2D kullanarak analitik düzlem X Y cinsinden hesaplamaktadır
    private Pose2d Set_ObstaclePoint(double Increment)
    {
      //Increment parametresi Ultrasonic'den çekilen ve robotun hareket doğrultusundaki çarpacağı objeye olan uzaklığı
      //bu uzaklığın obstacle noktasının koordinatları bulunması için robotun bulunduğu noktaya eklenmesi lazım
      //robotun önce rotasyonu bulunması lazım bu sayede eklenecek değerin ne kadarı X ne kadarı Y ekseninde olduğu bulunabilir
      Double Robot_Angle = Math.toRadians(Sensor_Integration.Get_Rotation_Angle());
      // Engelin X ve Y koordinatları için değişimi hesapla
      //arttırma noktası bileşenlerine ayrıldıktan sonra trigonometrik değerlere göre Obstacle noktasının tam değerleri temsil etmesi için Trigonometri kullanılıyor
      double deltaX = Current_Point.getX() + Increment * Math.cos(Robot_Angle);
      double deltaY = Current_Point.getY() +Increment * Math.sin(Robot_Angle);
      Pose2d Obstacle = new Pose2d(deltaX, deltaY, Current_Point.getRotation());
      //Değerler yerine yazıldıktan sonra Pose2D nesnesine aktarılarak metot bu nesneyi döndürüyor
      return Obstacle;
    }

    /*|END Title : ROBOT MANEVRA SİSTEMİ|*/  

    /*|Endregion : CAN MOTOR KONTROL  |*/
        /***************************/

       /* | Region : MOTOR DURUM İZLEME  |*/

       //Motorlara verilen güçlerin stabilitesi ve max motor gücü farkı
       //Periodic metot // Hem Teleop Hem de Autonomous için kullanılabilir
       void Motor_Stability(Double max_Difference)
       {
         double Current_Difference = Math.abs(Motor_Power_List.get(0) - Motor_Power_List.get(1));
         robot_Status = RobotStatus.DYNAMIC;
        // Current Difference Control
        //İLK ALTERNATİF
         if(Current_Difference > max_Difference)
         {
            double Extra_Difference = (Current_Difference -max_Difference ) / 2;
            Boolean extensial = Motor_Power_List.get(0) > Motor_Power_List.get(1);
            if(extensial)
            {
               Motor_Power_List.set(0,Motor_Power_List.get(0)-Extra_Difference) ;
            }
            else
            {
               Motor_Power_List.set(0,Motor_Power_List.get(0)+Extra_Difference) ;
            }
         }
       }
      //Periodic Metot // Hem Teleop Hem de Autonomous için kullanılabilir
      void Motor_Velocity_Equation(Double Desired_Speed, Double[] Current_Speed)
      {
         //ROBOTUN MOTORLARINA SABİT GÜÇ UYGULANARAK HAREKET SAĞLANMASI İÇİN ROBOTA ÖZEL DURUM EKLENMİŞTİR
         Motor_Power_List.set(0, Desired_Speed);
          Robot_Status_Situational(2);
      }
      public ArrayList<Double> PID_parametres(double k_Proportional, double k_Integral, double k_derivative)
      {
        PID_Coefficients.set(0, k_Proportional) ;
        PID_Coefficients.set(1, k_Integral) ;
        PID_Coefficients.set(2, k_derivative) ;
        return PID_Coefficients;
      }
       // ROBOTUN  MOTORLARINDAKİ HIZINA GÖRE MOTORUN BULUNDUĞU HALİ KONTROL EDİYORUZ
       //Periodic metot
      public void Robot_Status_Mechanical()
      {
        if(Situational == false)
        {
           if(Math.abs(Motor_Power_List.get(0)) < 0.025d && Math.abs(Motor_Power_List.get(1)) < 0.025d )
          {
             Motor_Power_List.set(0, 0d);
             Motor_Power_List.set(1, 0d);
            robot_Status = RobotStatus.IDLE;
          }
          else 
          {
              robot_Status = RobotStatus.DYNAMIC;
          }
        }
      }
      public void Robot_Status_Situational(int Situation)
      {
        if(Situation ==1 ) robot_Status =  RobotStatus.TURNING;
        else if(Situation ==2 ) robot_Status =  RobotStatus.CONSTANTPOWER;
        Situational = true;
      }
       /*|Endregion :  MOTOR DURUM İZLEME |*/
       /***************************/
 }
      

