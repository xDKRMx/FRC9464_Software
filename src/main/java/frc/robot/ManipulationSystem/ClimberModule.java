package frc.robot.ManipulationSystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
public  class ClimberModule {
     //Bu Class'deki işlevler oyun sonundaki zincire tutunma işlemlerini gerçekleştirmek için hazırlanacak altyapıya sahip
     //Bu class'deki işlemler Intake ve Shooter'dan bağımsız olacak ancak yine de asenkro olarak geliştirilmeye pek müsait değil çünkü bu buradaki algoritmalar hassas hesaplamar ve ince işçilik ile yapılması gerekiyor
     //Bu yüzden bu kodların öncesinde Intake ve Shooter Modülleri halledilecek eğer kki bir bakit boşluğu var o zaman asenkron bir şekilde geliştirilebilir
     //Bu Class bize bir nevi robotun tırmanma işlemlerini içerisinde barındıraağı ve bu sayede zincire tutunup dengesini kaybetmemesi için hassas işlemler içeren bir class olacak
     //Bu class'deki fonksiyonların ve bir çok değişkeninin public olarak tanımlanması veya OOP(Object oriented) mantığı ile encapsulation olarak tanımlanmış olması kodun ilerleyişi açısından daha iyi olur.
    //  /*****************/
      //Teleskopik kolu yönetecek motor ve solenoid
      // private CANSparkMax Telescobic_Motor_Left = new CANSparkMax(8,MotorType.kBrushless);
      // private CANSparkMax Telescobic_Motor_Right = new CANSparkMax(9,MotorType.kBrushless);
    public ClimberModule()
     {
        //  Telescobic_Motor_Left.setIdleMode(IdleMode.kBrake);
        //   Telescobic_Motor_Right.setIdleMode(IdleMode.kBrake);
     }
    public void Climb(String mode, String motor){
      // if(mode == "climb"){
      //   if(motor == "left")
      //   Telescobic_Motor_Left.set(0.5);
      //   else if(motor == "right")
      //   Telescobic_Motor_Right.set(0.5);
      // }
      // else if(mode =="descend"){
      //   if(motor == "left")
      //   Telescobic_Motor_Left.set(-0.5);
      //   else if(motor == "right")
      //   Telescobic_Motor_Right.set(-0.5);
      // }
    }
}
