// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.DriverSystem.*;
import frc.robot.ManipulationSystem.*;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
//Robot Java bizim kodların algoritmalarını birebir olarak tanımlayacağımız ve yazacağımız bir Java sayfası olmayacak,
// Robot.java bizim için modül modül olarak ayrılmış olan kodların algoritmalrın roobtun test kısmında teleop kısmında veya outonomous kısmında tek bir çatı altında bir araya getirilerek 
// Kodlar ve algoritmalar arasında bir çatı görevi görecek bu sayede bu yapı ile biz kodların algoritmalarını aynı sayfada oluşturmadığımızı için karmaşanın önüne geçeceğiz
//Bu sayfadaki fonksiyonlar içerisine Robot class'in altında composition veya polymorphism edilmiş class'ler ve Interface'lerin içerisindeki fonksiyonlar çağırılacak
 
// *************************************************//

 /* Header : Driver Sytem Class'leri ve Interfaceleri */
public MotorControllerModule Motor_Controller_Class = new  MotorControllerModule();
public InputProcessingModule Input_Processing_Class = new  InputProcessingModule(Motor_Controller_Class);
public TelemetryModule Telemetry_Class = new  TelemetryModule(Motor_Controller_Class);

//Bu class'i yorum satırına aldım çüknü bu class içerisindeki işlemlerin, fonksiyonların, değişkenlerin hepsini biz Robot.java Dışındaki class'lerden kullanacağız. Yani Robot.java'da bu class'i Instance'ını almaya gerek yok 
// public SensorIntegrationModule Sensor_Integration_Class = new  SensorIntegrationModule();

// *************************************************//
 /* Header : Manipulation Sytem Class'leri ve Interfaceleri */
// public ShooterModule Shooter_Class = new  ShooterModule();
public ClimberModule Climber_Class = new  ClimberModule(Motor_Controller_Class);
public ElevatorModule Elavator_Class = new  ElevatorModule();

 // *************************************************//
 /* Header  Diğer değişkenler ve sınıflar*/


 // *************************************************//

  public Robot() {

  }
  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {
  //Tanımlanan değişkenleri Smart Dashboard'a gönderme işlemi
    Telemetry_Class.Updating_Variables_Smart_Dashboard();
    //Robotun anlık durumunu sürekli kontrol etmemiz lazım ki robota bir işlem yapmak istediğimiz zaman yapılacak işlem robota ters düşmesin
      Motor_Controller_Class.Robot_Status_Mechanical();
  }


  @Override
  public void autonomousInit() {
    //Robotun otonom kısmında ilk başta neyi hazırlaması gerektiğini gösterir
    Motor_Controller_Class.Autonomous_Drive_Init();
  }

  @Override
  public void autonomousPeriodic() {
    //Robotun otonom kısmında periyodik ve adım adım yapması gereken işlemleri yönlendidiği kısımdır
    Motor_Controller_Class.Autonomous_Drive_Periodic();
  }

  @Override
  public void teleopInit() {
  }
  @Override
  public void teleopPeriodic() {
     Motor_Controller_Class.Follow_Periodic();
     //Buradan periyodik olarak Joysticklerimizdeki yön tuşlarının değerine göre robotumuzun motorlarına güç veriyoruz.
     Input_Processing_Class.Call_Driver_Periodic(true);
     //Tanımlanan değişkenleri Smart Dashboard'a gönderme işlemi
     Telemetry_Class.Updating_Variables_Smart_Dashboard();
     //Shooter sisteminin periyodik olarak çağırılma işlemi
      Motor_Controller_Class.Shooter_Module.Shooter_Periodic();
  }

  @Override
  public void testInit() {}


  @Override
  public void testPeriodic() {
  
  }
}
