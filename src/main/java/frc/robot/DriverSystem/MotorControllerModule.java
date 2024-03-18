package frc.robot.DriverSystem;


import frc.robot.ManipulationSystem.*;
import frc.robot.ManipulationSystem.ShooterModule.ShooterMotorStatus;

import java.util.ArrayList;
import java.util.Collections;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//Simülasyonu çalıştırmak için Keyboard Analog importu (Geçici)


//BU ENUM yapısı robotun motor durum kontrolünde kullanılacaktır ve ilerleyen aşamalarda sensör entegrasyonundaki veirlerle birlikte sağlanan değerler enum'daki robotun durumna göre şekillenecektir

public  class MotorControllerModule {
       public enum RobotStatus {
        DYNAMIC,
        IDLE,
        TURNING
       }
     //Bu class bir nevi bizim için bir çok class içerisinde kullanacağımız ve robottaki bir çok kodun temelini oluşturacak bir class. 
     // Bu class içerisindeki işlemler bir nevi robotun içerisindeki temel dinamikler diyebilirim bu temel dinamiklere örnek olarak sürüş, dönme hareket durma ivmeli hareket vs.
     //Bu class'deki fonksiyonların ve  bir çok değişkeninin public olarak tanımlanması veya OOP(Object oriented) mantığı ile encapsulation olarak tanımlanmış olması kodun ilerleyişi açısından daha iyi olur.
    /************************************************** */ 
    //Ek modüllerin tanımlaması
    public SensorIntegrationModule Sensor_Integration = new  SensorIntegrationModule(this);
    public ShooterModule Shooter_Module = new  ShooterModule(this);
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
     //Motorların Motor Stabilitesini sağlama Kontrolü
     /*
      Bu değişkenin işlevi şu eğer ki biz motorların sabit hızla gitmesini istiyorsak bunun için PID sistemli mMotor_Velocity_Equation() fonksiyonu kullanılacak. 
      Eğer ki biz motorların hızını sabit olmasını istemiyorsak bunun yerine motorların arasındaki  hız farkına göre bir yumuşatma işlemi uygulayacaksak Motor_Stability() sisteminde işlem yapılacak Motor_Stability() fonksiyonu kullanılacak.
       Biz bu fonksiyonları uygulamazsak bunun halinde motorlar arası  hız farkı çok olabilir bu da robotu çok hızlı döndürerek bozabilir
      */
      public boolean Stop_Rotate;
     //OTONOM KISMI İÇİN TANIMLANMACAK DEĞİŞKENLER
     private Timer timer = new Timer();
     private boolean reached_Angle = false;
     private double Initial_Angle_Difference = 999d;
     double TargetAngle;
     double CurrentAngle;
     //Otonom kısım Limelight'ları
     //ODOMETRİ HESABI İLE
     Pose2d Current_Point;
     Pose2d Setpoint;
     //Otonom Limelight
     public Double Y_April = 0d;
     public Double Initial_Y_April = 0d;
     public Double Limit_TY = -6.10d;
     //Robotun manevra işlemine girmesi için limit değişkenler
     private double Limit_Distance = 2d;
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
        //PID controlleri için başlangıç
        //zamanlayıcıyı sıfırla
        timer.reset();
        timer.start();
     }
     /***************************/

     
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
             if(robot_Status != RobotStatus.TURNING )
             {
                 // Mevcut motorun güçlerini al
               double currentLeftInput = Motor_Power_List.get(0);
               double currentRightInput = Motor_Power_List.get(1);
               // Joystick inputlarını al veya varsayılan olarak 0 kabul et
               double Speed_Input = Math.pow(joystick.getRawAxis(1),3);
               double Rotate_Input = Math.pow(joystick.getRawAxis(0),3);
               //Eşik değer kontrolü
               if(Math.abs(Speed_Input) < 0.008)Speed_Input = 0;
               if(Math.abs(Rotate_Input) < 0.008)Rotate_Input = 0;
              //  // Ramp algoritmasına göre Stabiliteyi sağlayacak bir ayarlama işlemi
               Double Absolute_Left_Motor_Power = rampMotorInput(currentLeftInput,Speed_Input,0.1f);
               Double Absolute_Right_Motor_Power = rampMotorInput(currentRightInput,Rotate_Input,0.1f);
                Motor_Power_List.set(0, Absolute_Left_Motor_Power);
                Motor_Power_List.set(1, Absolute_Right_Motor_Power );
             }
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
          //System.out.println(Motor_Power_Control + " " + robot_Status);
          Main_Robot_Drive.arcadeDrive( Motor_Power_List.get(0),Motor_Power_List.get(1),false); 
         //  System.out.println(Left_Leader.get() + " Left motor " + Right_Leader.get() + " right ");
         
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
       //Otonom kodda robotun orta sahaya kadar yaklaşmada komutu verecek tag boolean'İ
       private Boolean Tag_Control;
       public void Autonomous_Drive_Periodic() {
        /*OTONOMDA YAPILACAK İŞLEMLER */
         // ilgili noktaya dönme, döndükten sonra hopörlöre atış işlemi ardından tekrar eski başlangıç açısına dönme
           //eski başlangıç açısına yakın bir değere dönüp taksi yapma taksi yapılıp belli bir mesafe katedildikten sonra kendi ekseninde dönüp ilgili kaynak noktasının Apriltagini algılma 
           //Kaynak noktasının apriltag'i algılandıktan sonra limelight dökümentasyonundaki mesafe ölçümü ile uzaklık bulunduktan sonra o uzaklğının birazcık daha az değeri ile robota gitmesi için bir setpoint noktası belirleyip oraya yöneltme
           //Robotun her frame otonomdaki işlemleri matematiksel olarak değerlendirip onu koordinat sistemine göre değerlendireceğiz
            //Robotun varolan açısını bulma
          /* CurrentAngle =Sensor_Integration.Get_Rotation_Angle(); */
            CurrentAngle =Sensor_Integration.Get_Rotation_Angle();
           if(timer.get() < 2)
           {
            //İlk 2 saniye boyunca robot kendini notayı atacak şekilde ayarladıktan sonra robotun içindeki notayı hopörlöre atma işlemi
            //Notanın robot içinde olup olmaması Touch sensörü ile kontrol ediliyor
             boolean Note_In_Robot = Sensor_Integration.Note_Touch_Control();
             if(Note_In_Robot) Shooter_Module.Shoot_Subsystem("Speaker","Shooter");
             else Shooter_Module.SlowDown_Motor_Power("Shooter");
          
           }
           else if (timer.get() >= 2 && timer.get() <= 4.5) {
               if(Shooter_Module.Shooter_Status == ShooterMotorStatus.Dynamic) Shooter_Module.SlowDown_Motor_Power("Shooter");
               if(reached_Angle == false)
               {
               //Robotun varolan açısına geri dönmesi 
               if( Initial_Angle_Difference==999d)
                {
                  TargetAngle =Math.toDegrees(CurrentAngle) + 180;
                  if(TargetAngle >360 ) TargetAngle -= 360;
                  if(TargetAngle < 0 ) TargetAngle += 360;
                  TargetAngle = Math.toRadians(TargetAngle);
                  Initial_Angle_Difference  =  Math.abs(CurrentAngle - TargetAngle);
                }
               if(!Stop_Rotate)
               {
                if(Math.toDegrees(Math.abs(CurrentAngle - TargetAngle))>  1d )
                {
                    double Turning_Speed =  (Math.abs(TargetAngle - CurrentAngle ) / Initial_Angle_Difference ) ;
                    Rotate_Robot(Turning_Speed , (TargetAngle - CurrentAngle) > 0 ? true : false );
                }
               }
                if(Math.toDegrees(Math.abs(CurrentAngle - TargetAngle)) < 1d )
                {
                      Initial_Angle_Difference = 999d;
                      Stop_Rotating();
                      reached_Angle = true;
                      Motor_Power_List.set(1, 0d);
                }
               }
           }
           else if(timer.get() >=4.5 && timer.get() < 15)
           {
             Motor_Power_List.set(0, -0.2d); 
             Stop_Rotate = false;
             if(timer.get() >=4.5 && timer.get() <=7)
             {
                Tag_Control =  VisionProcessing.Scan_Apriltag() == 14 || VisionProcessing.Scan_Apriltag() == 13 ? true : false;
                Motor_Power_List.set(0, 0.2d);
             } 
             else 
             {
              if(!Tag_Control) 
              {
                 Motor_Power_List.set(0, 0d); 
                 Motor_Power_List.set(1, 0d);
              }
              else
              {
                if(Initial_Y_April == 999d) Y_April = VisionProcessing.getTargetOffsetY();
                Y_April = VisionProcessing.getTargetOffsetY();
               if(Y_April < Limit_TY)
               {
                Double Power_Ratio = ((Y_April) / Initial_Y_April)/ 5;
                Motor_Power_List.set(0, Power_Ratio); 
               }
               else
               {
                 Motor_Power_List.set(0, 0d); 
                 Motor_Power_List.set(1, 0d);
               }
              }
             } 
           }
           Main_Robot_Drive.arcadeDrive( Motor_Power_List.get(0),Motor_Power_List.get(1),false); 
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
              Robot_Status_Situational();
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
      // double Initial_Distance = 999d;
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
             System.out.println(Current_Distance + " distance");
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
         /****************** */
    }
    /*|END Title : ROBOT MANEVRA SİSTEMİ|*/  

    /*|Endregion : CAN MOTOR KONTROL  |*/
        /***************************/

       /* | Region : MOTOR DURUM İZLEME  |*/
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
          else  robot_Status = RobotStatus.DYNAMIC;
        }
      }
      public void Robot_Status_Situational()
      {
         robot_Status =  RobotStatus.TURNING;
        Situational = true;
      }
       /*|Endregion :  MOTOR DURUM İZLEME |*/
       /***************************/
 }
      

