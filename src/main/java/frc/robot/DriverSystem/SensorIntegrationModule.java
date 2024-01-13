package frc.robot.DriverSystem;

public  class SensorIntegrationModule {
      //Bu class Motor Controller ve Input Processing Modüllerindeki işlemlerin devamı niteliğindedir
      //Not: bu Class'i geliştirmeye başladığımız zaman bir daha MotorControllerModule veya InputProcessingModule'lerine geri dönmeyeceğiz gibi algılanmasın. Burada çakışan kodlar olursa her türlü iyileştirilmeler yapılacaktır
     // Bu kodun temel işlevi Robotun sahip olduğu sensörlerin tanımlanması ve bu sensötlerle yapılacak alası basit algoritmaların gereksiz yere Robot.Java üzerinden tanımlanması yerimne burada tanımlanacak
     //Örnek olarak mesela Autonomous için Gyro kullanılarak başlangıç noktasından robota en yakındaki amfiye veya hopörlöre olan ortalama mesafe hesaplanabilir 
     // Bu class bir nevi Sensörler için bir Base olacak yani başka class'lerde özellikle Manıpulation sytem içerisinde bu class'de tanımlanmış olan sensörlerden oldukça yararlanacağız o yüzden bu class diğer class'lere neredeyse kusursuz bir imkan sağlamalı Sensör konusunda
     //Bu class'deki fonksiyonların ve bir çok değişkeninin public olarak tanımlanması veya OOP(Object oriented) mantığı ile encapsulation olarak tanımlanmış olması kodun ilerleyişi açısından daha iyi olur.
     public SensorIntegrationModule()
     {
        
     }
}
