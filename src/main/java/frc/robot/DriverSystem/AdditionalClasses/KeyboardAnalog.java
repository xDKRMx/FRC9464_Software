package frc.robot.DriverSystem.AdditionalClasses;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.util.HashSet;
import javax.swing.JFrame;

public  class KeyboardAnalog  extends JFrame{
     private double Speed_amount_LEFT = 0;
     private double Speed_amount_RIGHT = 0;
    private final HashSet<Integer> pressedKeys = new HashSet<>();

   public KeyboardAnalog()
   {
       setSize(300, 200);
        setVisible(true);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        //Eğer ki bilgisayarımıza herhangi bir Joystick bağlı değilse klavyemizi Joystic olarak varsayıp motorlara gücü klavyemizden atanılan tuşlarla oluşturacağız
        addKeyListener(new KeyAdapter() {
          @Override
          public void keyPressed(KeyEvent e) {
              pressedKeys.add(e.getKeyCode());
              updateSpeed();
          }

          @Override
          public void keyReleased(KeyEvent e) {
              pressedKeys.remove(e.getKeyCode());
              updateSpeed();
          }
      });
   }
     //Periodic Metot
     public void periodic_KeyListener()
     {
        setFocusable(true);
        requestFocusInWindow();
     }

     private void updateSpeed() {
        //************************//
        //ARCADE DRİVE İÇİN UPDATE SPEED İŞLEMİ
        // Motorların HIZINI ayarlama işlemi
        // if (pressedKeys.contains(KeyEvent.VK_W))  Speed_amount_LEFT = increaseSpeed(Speed_amount_LEFT);
        // else if (pressedKeys.contains(KeyEvent.VK_S))  Speed_amount_LEFT = decreaseSpeed(Speed_amount_LEFT);
        // else Speed_amount_LEFT = 0;
    
        // // Robotun DÖNÜŞÜNÜ ayarlama işlemi
        // if (pressedKeys.contains(KeyEvent.VK_A))  Speed_amount_RIGHT =0.2f; 
        // else if (pressedKeys.contains(KeyEvent.VK_D)) Speed_amount_RIGHT = -0.2f; 
        //  else Speed_amount_RIGHT = 0;

          //************************//


        //TANK DRİVE İÇİN UPDATE SPEED İŞLEMİ
         //************************//
         /*Robotun SOL motor hızını ayarlama işlemi*/
      if (pressedKeys.contains(KeyEvent.VK_W))  Speed_amount_LEFT = increaseSpeed(Speed_amount_LEFT);
      else if (pressedKeys.contains(KeyEvent.VK_S))  Speed_amount_LEFT = decreaseSpeed(Speed_amount_LEFT);
       else Speed_amount_LEFT = 0;
    
        /*Robotun SAĞ motor hızını ayarlama işlemi*/
       if (pressedKeys.contains(KeyEvent.VK_UP))  Speed_amount_RIGHT = increaseSpeed(Speed_amount_RIGHT); 
       else if (pressedKeys.contains(KeyEvent.VK_DOWN)) Speed_amount_RIGHT = decreaseSpeed(Speed_amount_RIGHT); 
       else Speed_amount_RIGHT = 0;

          //************************//
    }
    
    private double increaseSpeed(double currentSpeed) {
        return Math.min(currentSpeed + 0.05, 1f); // Maksimum hızı 1 olarak sınırla
    }
    
    private double decreaseSpeed(double currentSpeed) {
        return Math.max(currentSpeed - 0.05, -1f); // Minimum hızı -1 olarak sınırla
    }

     /*Burası tam istenildiği gibi çalışmadığı için yorum satırına alındı */ 
    // private double reduceToZero(boolean Is_It_Left) {
    //     // if(Is_It_Left)
    //     // {
    //     //   while(Speed_amount_LEFT != 0 && isDecreasingLeft)
    //     //   {
    //     //     Float change = Speed_amount_LEFT > 0 ? -0.1f : 0.1f;
    //     //     Speed_amount_LEFT += change ;
    //     //   }
    //     // }
    //     // else
    //     // {
    //     //      while(Speed_amount_RIGHT != 0 && isDecreasingRight)
    //     //      {
    //     //       Float change = Speed_amount_RIGHT > 0 ?-0.1f : 0.1f;
    //     //       Speed_amount_RIGHT += change ;
    //     //      }
    //     // }
    //     return 0; // Eğer hız zaten 0 ise, değişiklik yapma
    // }

     public double Motor_Speed_Key_Analog(String Left_or_Right)
     {
        //Buradan hangi numaralı tuşa basıldığını algılayıp ona göre değer döndüreceğiz
      if(Left_or_Right == "Left") return Speed_amount_LEFT;
      else if(Left_or_Right == "Right")return Speed_amount_RIGHT;
      else  return 0;
    }

   
}
