package us.ihmc.etherCAT.examples;

import us.ihmc.etherCAT.master.EtherCATRealtimeThread;
import us.ihmc.etherCAT.slaves.IHMCBlueBox;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;

public class IMUExample extends EtherCATRealtimeThread
{
   private final IHMCBlueBox blueBox = new IHMCBlueBox(0, 0);
   private int counter = 0;
   
   public IMUExample()
   {
      super("eth2", PriorityParameters.MAXIMUM_PRIORITY, new MonotonicTime(0, 1000000), true, 100000);
      registerSlave(blueBox);
   }
   
   @Override
   protected void deadlineMissed()
   {
      
   }

   @Override
   protected void doControl()
   {
      if(counter++ % 1000 == 0)
      {
         System.out.println("Q: " + blueBox.getIMU0qs() + " " + blueBox.getIMU0qx() + " " + blueBox.getIMU0qy() + " " + blueBox.getIMU0qz());
      }
   }
   
   public static void main(String[] args)
   {
      IMUExample imuExample = new IMUExample();
      imuExample.start();
      imuExample.join();
   }

   @Override
   protected void workingCounterMismatch(int expected, int actual)
   {

   }

   @Override
   protected void datagramLost()
   {
      
   }
}
