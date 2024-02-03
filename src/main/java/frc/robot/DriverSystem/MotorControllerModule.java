package frc.robot.DriverSystem;



import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//Simülasyonu çalıştırmak için Keyboard Analog importu (Geçici)
import frc.robot.DriverSystem.AdditionalClasses.KeyboardAnalog;
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
     //OTONOM KISMI İÇİN TANIMLANMACAK DEĞİŞKENLER
     private Timer timer = new Timer();
     private boolean Is_Point_Setted = false;
     private boolean reached_Setpoint = false;
     private boolean reached_Angle = false;
     private double Initial_Angle_Difference = 999d;
     Pose2d Setpoint ;
    
     double TargetAngle;
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
     public double Get_Timer()
     {
      double Current_time = timer.get();
      return Current_time;
     }
      /* | End Region : MOTOR VERİLERİ ÇEKME  |*/


     /***************************/
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
               double currentLeftInput = Power_Of_Each_Motors.get(0);
               double currentRightInput = Power_Of_Each_Motors.get(1);
               // Joystick inputlarını al veya varsayılan olarak 0 kabul et
               double leftJoystickInput = -joystick.getRawAxis(1);
               double rightJoystickInput =  -joystick.getRawAxis(5);
               //Eşik değer kontrolü
               if(Math.abs(leftJoystickInput) < 0.07)leftJoystickInput = 0;
               if(Math.abs(rightJoystickInput) < 0.07)rightJoystickInput = 0;
               // Ramp algoritmasına göre Stabiliteyi sağlayacak bir ayarlama işlemi
               Double Absolute_Left_Motor_Power = rampMotorInput(currentLeftInput,leftJoystickInput,0.1f);
               Double Absolute_Right_Motor_Power = rampMotorInput(currentRightInput,rightJoystickInput,0.1f);
                Power_Of_Each_Motors.set(0, Absolute_Left_Motor_Power);
                Power_Of_Each_Motors.set(1,Absolute_Right_Motor_Power );
             }
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
          //System.out.println(Motor_Power_Control + " " + robot_Status);
           Main_Robot_Drive.tankDrive( Power_Of_Each_Motors.get(0),Power_Of_Each_Motors.get(1));

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
         //  saniye ileri gitme ve setpoint belirleme
           if (timer.get() < 2) {
               // İleri git
               Power_Of_Each_Motors.set(0, 0.5d);
               Power_Of_Each_Motors.set(1, 0.5d);
                Main_Robot_Drive.tankDrive(Power_Of_Each_Motors.get(0), Power_Of_Each_Motors.get(1));
           } 
           else {
               // Rastgele setpoint belirle // Şu anlık görüntü işleme algoritması yazılmadığından robotun gitmesi için rastgele bir set point belirliyoruz ve robotun motor hızını PID ile kontrol ediyoruz
               //İlk başta robotumuzun default POSE2D class'ini kullanarak posizyon bilgilerini çekiyoruz
               Pose2d Current_Position = TelemetryModule.Pose_Sendable.GetPose(); 
               Double Current_X_Position = Current_Position.getX();
               Double Current_Y_Position = Current_Position.getY();
               // sürekli yeni bir Setpoint noktası oluşturmaması için bir kereliğine mahsus robotun en az 5 en fazla 10 metre ilerisinde vye gerisinde olacak şekilde hem ,X hem Y ekseninde, bir random setpoint oluşturuyoruz
               if(Is_Point_Setted == false)
               {
                  Setpoint = SetRandomPoint(Current_X_Position, Current_Y_Position);
                  Is_Point_Setted = true;
               }
               //robotun normal bulunduğu konum ile setpoint noktası arasındaki doğrunun eğimine göre (tana) target angle değerini hesaplıyoruz
               TargetAngle = CalculateAngle(Current_Position,Setpoint);
               //Robotun şu an Gyro benzeri bir sensörler entegre edilmediği yönünü çekmekte zorlanıyoruz sanal ortamda
               //alternatif olarak da Pose2D'de kullanılan rotation2D kullanıyoruz
               //Rotation2D ve Pose2D maç sırasında bizim hesaplayabileceğimiz veri yapılarına sahip değil biz bunları sadece simülasyon sırasında test etmek için kullanabiliriz
               //gerçek robotta testleri koordinat sistemini sensörlerden Encoder sensörünü kullanarak 
               //Yön olarak NavX-MXP sensörünü kullanarak
               //random Setpoint noktası oluşturmak yerine de Limelight 3 ile görüntü işleme sistemi sayesinde en yakındaki AMP veya Hopörlörü bir setpoint noktası olarak atayacağız
               //robotun eğer ki sahip olduğu bir Intake sistemi varsa yerdeki notayı setpoint olarak algılayacak
               //Ardından da buradaki kendi konumu ile gideceği setpoint arasındaki mesafeyi ve buna bağlı olarak error mesafesini UltraSonic sensörle sağlayacak
               double CurrentAngle =Current_Position.getRotation().getDegrees();
               //robotun o andaki mevcut açısı  -180 180 derece dışındaysa esas ölçüsü alınır
               while (CurrentAngle > 180) CurrentAngle -= 360;
               while (CurrentAngle < -180) CurrentAngle += 360;
               //Başlangıçta sahip olduğu yön yani açı değeri ile setpoint ile Current point yani robotun konumu arasındaki açının farkı hesaplanır
               if( Initial_Angle_Difference==999d)
               {
                 Initial_Angle_Difference  =  Math.abs(CurrentAngle - TargetAngle );
               }
               //Lineer interpolasyon sistemi ile robotun yönünü target angle'a yaklaştırmak maksadıyla bir dönme hızı veriliyor.
               //Lineer İnterpolasyon : İKİ sayı arasındaki farkı yine farklı iki sayı arasındaki değere indirgemek. Mesela 40 - 0 arasındaki sayıları 1 ile 0'a indirmek gibi 40 ise sayı değeri 1,  0 ise  0 , 20 ise 0.5 gibi. 
                double Turning_Speed = ( Initial_Angle_Difference < -90d || Initial_Angle_Difference > 90d ) ? (Math.abs(CurrentAngle - TargetAngle ) / Initial_Angle_Difference )  : (Math.abs(CurrentAngle - TargetAngle) / Initial_Angle_Difference ) ;
              if(Math.abs(CurrentAngle - TargetAngle) > 0.01d && !reached_Setpoint && !reached_Angle)
              {
                //arardaki açı değeri neredeyse aynı olana kadar robotu en kısa yoldan döndür
               Rotate_Robot(Turning_Speed, (CurrentAngle - TargetAngle) > 0 ? false : true );
              }
              else
              {
                //koşul sağlandığında robotu döndürmeyi bırak ve robotun motorlarına bu sefer istenilen Setpoint noktasına gitmesi için güç ver
                Stop_Rotating();
                if(reached_Angle == false) reached_Angle = true;
              }
              if(reached_Angle)
              {
                //Robotun motorlarına  güç ver
                 Autonomous_Set_Robot(TargetAngle,Current_Position,Setpoint);
              }
              Pose2dSendable.field2.setRobotPose(Setpoint);
           }
       }
        private Pose2d SetRandomPoint(double X_position, double Y_Position)
        {
          //Rnadom Setpoint noktaıs oluşturma algoritması 
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
             Pose2d Setpoint = new Pose2d(Set_Point_X_Position, Set_Point_Y_Position, Default_Rotation);
             return Setpoint;
        }
        public double CalculateAngle(Pose2d CurrentPoint, Pose2d SetPoint )
        { 
          //burada iki nokta arasındaki açı farkını trigonometri ile hesaplıyoruz
          // Setpoint'e göre hedef açıyı hesapla
          double targetAngle = Math.toDegrees(Math.atan2(SetPoint.getY() - CurrentPoint.getY(),
          SetPoint.getX() - CurrentPoint.getX()));
          return targetAngle;
        }

       //Otonom Motor Drive Kodu
       //Başlangıçdaki erro yani iki nokta arasındaki fark absürt bir değer girilerek bir kereye mahsus ilk fark hesaplanmak istenmiştir
       private double Initial_Error = 999d; 
       private void Autonomous_Set_Robot(double Difference_Angle,Pose2d CurrentPoint,Pose2d setPoint) {
        //SetPoint ile CurrentPoint arasındaki mesafeyi yani error'ü hesaplamak (Dik üçgen yöntemi ile hesaplanmaktadır)
        double x_distance = setPoint.getX() - CurrentPoint.getX();
        double y_distance = setPoint.getY() - CurrentPoint.getY();
        double error = Math.sqrt(x_distance * x_distance + y_distance * y_distance);
        
        if(Initial_Error == 999d) Initial_Error = error;
        //Roobtun motorlarına verilecek gücün yine Lineer Interpolasyon ile eror mesafesine bağlı olarak verilmesi
        double Power = ( Difference_Angle < -90d || Difference_Angle > 90d ) ?(error / Initial_Error)  :  (error / Initial_Error) ; 
        if (Math.abs(error) < 0.1d) { // Hedefe yaklaştığında ve eşik değer geçildiğinde artık direkt durma komutu yazılmış
           Power_Of_Each_Motors.set(0, 0d);
            Power_Of_Each_Motors.set(1,0d);
        } else {
          //Hedef ile arasındaki error mesafesi hala eşik değer üstündeyse robotun motorlarına power oranında güç ver 
            Power_Of_Each_Motors.set(0, Power);
            Power_Of_Each_Motors.set(1, Power);
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
       //Periodic metot // Hem Teleop Hem de Autonomous için kullanılabilir
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
       //Periodic metot
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
       /***************************/
 }
      

