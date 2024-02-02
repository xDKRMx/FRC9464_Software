package frc.robot.DriverSystem.AdditionalClasses;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.util.HashSet;
import javax.swing.JFrame;

import frc.robot.DriverSystem.MotorControllerModule;
import frc.robot.DriverSystem.MotorControllerModule.RobotStatus;

public  class KeyboardAnalog  extends JFrame{
    // POWER DEĞERLERİ 
     private double Power_amount_LEFT = 0;
     private double Power_amount_RIGHT = 0;
     private final HashSet<Integer> pressedKeys = new HashSet<>();
     // Ek modüllerin tanımlanması
      private MotorControllerModule Motor_Controller;
    
   public KeyboardAnalog(MotorControllerModule _Motor_Controller)
   {
        Motor_Controller = _Motor_Controller;
        setSize(300, 200);
        setVisible(true);
         setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        //Eğer ki bilgisayarımıza herhangi bir Joystick bağlı değilse klavyemizi Joystic olarak varsayıp motorlara gücü klavyemizden atanılan tuşlarla oluşturacağız
        addKeyListener(new KeyAdapter() {
          @Override
          public void keyPressed(KeyEvent e) {
              pressedKeys.add(e.getKeyCode());
              Key_Board_Input();
          }

          @Override
          public void keyReleased(KeyEvent e) {
              pressedKeys.remove(e.getKeyCode());
              Key_Board_Input();
          }
      });
   }
     //Periodic Metot
     public void periodic_KeyListener()
     {
         setFocusable(true);
     }

     private void Key_Board_Input() {
        //************************//
        //ARCADE DRİVE İÇİN UPDATE SPEED İŞLEMİ
        // Motorların HIZINI ayarlama işlemi
        // if (pressedKeys.contains(KeyEvent.VK_W))  Power_amount_LEFT = increaseSpeed(Power_amount_LEFT);
        // else if (pressedKeys.contains(KeyEvent.VK_S))  Power_amount_LEFT = decreaseSpeed(Power_amount_LEFT);
        // else Power_amount_LEFT = 0;
    
        // // Robotun DÖNÜŞÜNÜ ayarlama işlemi
        // if (pressedKeys.contains(KeyEvent.VK_A))  Power_amount_RIGHT =0.2f; 
        // else if (pressedKeys.contains(KeyEvent.VK_D)) Power_amount_RIGHT = -0.2f; 
        //  else Power_amount_RIGHT = 0;

          //************************//
        //TANK DRİVE İÇİN UPDATE SPEED İŞLEMİ
         //************************//

         //Bu if koşulunun sebebi eğer ki robotun motorlarındaki kontrol Motor_Velocity_Equation() metodu tarafından sağlanıyorsa bu metot iki motorun da güçlerini eşitlemek istediği için kalvye analogundan gelen değerler çakışmaması için koşul koyulmuştur
        if( Motor_Controller.robot_Status != RobotStatus.CONSTANTPOWER)
        {
             /*Robotun SOL motor hızını ayarlama işlemi*/  
             if (pressedKeys.contains(KeyEvent.VK_W))  Power_amount_LEFT = increaseSpeed(Power_amount_LEFT);
             else if (pressedKeys.contains(KeyEvent.VK_S))  Power_amount_LEFT = decreaseSpeed(Power_amount_LEFT);
             else 
             {
                if(Motor_Controller.robot_Status != RobotStatus.TURNING)
               Power_amount_LEFT = 0;
             }
           
               /*Robotun SAĞ motor hızını ayarlama işlemi*/
              if (pressedKeys.contains(KeyEvent.VK_UP))  Power_amount_RIGHT =increaseSpeed(Power_amount_RIGHT); 
              else if (pressedKeys.contains(KeyEvent.VK_DOWN)) Power_amount_RIGHT = decreaseSpeed(Power_amount_RIGHT); 
             else 
             {
               if(Motor_Controller.robot_Status != RobotStatus.TURNING )
                  Power_amount_RIGHT = 0;
             }
        }
    

          //************************//
      // KLAVYEDE DİĞER TUŞLARA BASILDIĞI ZAMAN GERÇEKLEŞECEKLER
      if(pressedKeys.contains(KeyEvent.VK_D))
      {
        Motor_Controller.Motor_Power_Control = "PID";
      }else
      {
        Motor_Controller.Motor_Power_Control = "Stability";
      }
      
       if(pressedKeys.contains(KeyEvent.VK_RIGHT))
      {
       
        Motor_Controller.Rotate_Robot(1d,true);
        
      }else if(pressedKeys.contains(KeyEvent.VK_LEFT))
      {
        Motor_Controller.Rotate_Robot(1d,false);
      }
      else {
         Motor_Controller.Stop_Rotating();
      }
    }


    private double increaseSpeed(double currentSpeed) {
        return Math.min(currentSpeed + 0.05, 0.5d); // Maksimum hızı 1 olarak sınırla
    }
    
    private double decreaseSpeed(double currentSpeed) {
        return Math.max(currentSpeed - 0.05, -0.5d); // Minimum hızı -1 olarak sınırla
    }
     public double Motor_Speed_Key_Analog(String Left_or_Right)
     {
        //Buradan hangi numaralı tuşa basıldığını algılayıp ona göre değer döndüreceğiz
      if(Left_or_Right == "Left") return Power_amount_LEFT;
      else if(Left_or_Right == "Right")return Power_amount_RIGHT;
      else  return 0;
    }

   
}
