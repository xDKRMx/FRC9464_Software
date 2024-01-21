package frc.robot.DriverSystem.AdditionalClasses;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.util.HashSet;
import java.util.Set;
import javax.swing.JFrame;

public  class KeyboardAnalog  extends JFrame{
     private double Speed_amount_LEFT = 0;
     private double Speed_amount_RIGHT = 0;
    private final Set<Integer> pressedKeys = new HashSet<>();

   public KeyboardAnalog()
   {
       setSize(300, 200);
        setVisible(true);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
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
        //Eğer ki bilgisayarımıza herhangi bir Joystick bağlı değilse klavyemizi Joystic olarak varsayıp motorlara gücü klavyemizden atanılan tuşlarla oluşturacağız
         // klavyeden gelen tuş basımlarını sürekli olarak kontrol et
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

   private void updateSpeed() {
        //Sol motoru güç verebilecek herhangi bir tuşa basılmış mı
        if (pressedKeys.contains(KeyEvent.VK_W)) Speed_amount_LEFT = 1;
        else if (pressedKeys.contains(KeyEvent.VK_S))  Speed_amount_LEFT = -1;
        else Speed_amount_LEFT = 0;

        //Sağ motoru güç verebilecek herhangi bir tuşa basılmış mı
        if (pressedKeys.contains(KeyEvent.VK_UP))   Speed_amount_RIGHT = 1;        
        else if (pressedKeys.contains(KeyEvent.VK_DOWN))  Speed_amount_RIGHT = -1;
        else  Speed_amount_RIGHT = 0;
        //Hız kontrol
        
    }

     public double Motor_Speed_Key_Analog(String Left_or_Right)
     {
        //Buradan hangi numaralı tuşa basıldığını algılayıp ona göre değer döndüreceğiz
      if(Left_or_Right == "Left") return Speed_amount_LEFT;
      else if(Left_or_Right == "Right")return Speed_amount_RIGHT;
      else  return 0;
    }

   
}
