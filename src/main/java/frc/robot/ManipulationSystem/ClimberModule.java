package frc.robot.ManipulationSystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.DriverSystem.MotorControllerModule;
import frc.robot.DriverSystem.MotorControllerModule.RobotStatus;

public  class ClimberModule {
     //Bu Class'deki işlevler oyun sonundaki zincire tutunma işlemlerini gerçekleştirmek için hazırlanacak altyapıya sahip
     //Bu class'deki işlemler Intake ve Shooter'dan bağımsız olacak ancak yine de asenkro olarak geliştirilmeye pek müsait değil çünkü bu buradaki algoritmalar hassas hesaplamar ve ince işçilik ile yapılması gerekiyor
     //Bu yüzden bu kodların öncesinde Intake ve Shooter Modülleri halledilecek eğer kki bir bakit boşluğu var o zaman asenkron bir şekilde geliştirilebilir
     //Bu Class bize bir nevi robotun tırmanma işlemlerini içerisinde barındıraağı ve bu sayede zincire tutunup dengesini kaybetmemesi için hassas işlemler içeren bir class olacak
     //Bu class'deki fonksiyonların ve bir çok değişkeninin public olarak tanımlanması veya OOP(Object oriented) mantığı ile encapsulation olarak tanımlanmış olması kodun ilerleyişi açısından daha iyi olur.
     /*****************/
      //Ek modüllerin tanımlaması
      private MotorControllerModule Motor_Control_Module;
      //PID dengesi
      private PIDController Heigh_Balance;
      private PIDController Torque_Balance;
      //Zincire tırmanma öncesi başlangıç değerleri
      public Boolean Climbing_Started = false;
      Double[] Initial_Axis_Angles;
      float Default_Height;
      //Teleskopik kolu yönetecek motor ve solenoid
     // private CANSparkMax Telescobic_Motor = new CANSparkMax(6,MotorType.kBrushless);
      //Climbing Değişkenleri
      //Limit
      private Double Limit_Heigt = 20d; // cm cinsinden;
      Double Limit_Deviation = 2d; // cm cinsinden;
      //Current ve durum değişkenleri
      Double[] Current_Deviation;
      Double Current_Height = 0d;
      Double Climber_Motor_Power = 0d;
      Boolean Is_Robot_Climbing = false;
      
     public ClimberModule(MotorControllerModule _Motor_Control_Module)
     {
         Motor_Control_Module = _Motor_Control_Module;
     }

     public void Init_Climbing()
     {
         if(Climbing_Started == false) 
         {
          Climbing_Started = true;
          //Sensör entegrasyon modülündeki Navx Sensörünü kullanarak robotun zincire çıkmadan önceki açı değerlerini kontrol ediyoruz 
          //Eğer ki robot yukarı doğru Teleskobik kol ile yukarı çıkarken dengesinde sapma yaşarsa biz bu sapmayı açı cinsinden yaklayacağımız için bir karşılaştırma yapmamız lazım ve buradan çıkacak değerler de robotun dengesindeki sapmayı gösterecek
          Initial_Axis_Angles = Motor_Control_Module.Sensor_Integration.Three_Axis_Rotation();
          Default_Height = Motor_Control_Module.Sensor_Integration.get_Height();
         }
     }

     public void Climbing()
     {
        if(Climbing_Started ) 
        {
          if( Math.abs(Current_Height)> 0.05)
          {
            Elevator_System();
           Balance_Control();
          }
          else
          {
             //  Telescobic_Motor.setIdleMode(IdleMode.kBrake);
            //   Telescobic_Motor.set(0);
          }
        }
        else
        {

        }
     }
     public void Elevator_System()
     {
        Current_Height = Double.valueOf(Motor_Control_Module.Sensor_Integration.get_Height());
       Climber_Motor_Power = (Default_Height - Current_Height) / Current_Height;
        Climber_Motor_Power = Math.max(-0.5, Math.min(0.5, Climber_Motor_Power));
       // Telescobic_Motor.set(Climber_Motor_Power);
     }
     public void Balance_Control()
     {
      Double[] Current_Axis_Angles = Motor_Control_Module.Sensor_Integration.Three_Axis_Rotation();
      for(int a = 0 ; a < 3 ; a++)
      {
       Current_Deviation[a] = Current_Axis_Angles[a] - Initial_Axis_Angles[a];
       if(Math.abs(Current_Deviation[a] ) >Limit_Deviation)
       {
        Double Balance_Power =  Limit_Deviation / Current_Deviation[a]  ;
        Motor_Control_Module.Set_Power(-Balance_Power, -Balance_Power);
        Motor_Control_Module.robot_Status = RobotStatus.TURNING;
       }
       else
       {
        Motor_Control_Module.Stop_Rotating();
       }
      }
     }
}
