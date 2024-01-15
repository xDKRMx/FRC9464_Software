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
public InputProcessingModule Input_Processing_Class = new  InputProcessingModule();
public SensorIntegrationModule Sensor_Integration_Class = new  SensorIntegrationModule();
public TelemetryModule Telemetry_Class = new  TelemetryModule();

// *************************************************//
 /* Header : Manipulation Sytem Class'leri ve Interfaceleri */
public IntakeModule Intake_Class = new  IntakeModule();
public ShooterModule Shooter_Class = new  ShooterModule();
public ClimberModule Climber_Class = new  ClimberModule();
public ElevatorModule Elavator_Class = new  ElevatorModule();

 // *************************************************//
 /* Header : Driver Sytem Class'leri ve Interfaceleri */


 // *************************************************//

  public Robot() {
   System.out.println("merhaba");
  }


  @Override
  public void robotInit() {

  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {}


  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {}


  @Override
  public void testPeriodic() {}
}
