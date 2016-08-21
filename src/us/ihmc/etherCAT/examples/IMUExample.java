package us.ihmc.etherCAT.examples;

import us.ihmc.etherCAT.master.EtherCATRealtimeThread;
import us.ihmc.etherCAT.slaves.IHMCBlueBox;
import us.ihmc.etherCAT.slaves.ihmc.IHMCEtherCATIMU;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;

public class IMUExample extends EtherCATRealtimeThread
{
   private final IHMCEtherCATIMU ihmcEtherCATIMU = new IHMCEtherCATIMU(3, 0);
   private final IHMCBlueBox ihmcBlueBox = new IHMCBlueBox(3, 1);
   private int counter = 0;
   
   public IMUExample()
   {
      super("enx9cebe830b0db", PriorityParameters.MAXIMUM_PRIORITY, new MonotonicTime(0, 1000000), true, 100000);
      registerSlave(ihmcEtherCATIMU);
      registerSlave(ihmcBlueBox);
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
         System.out.println("BusVoltage: " + ihmcEtherCATIMU.getBusVoltage() + " Q: " + ihmcEtherCATIMU.getIMU0qs() + " " + ihmcEtherCATIMU.getIMU0qx() + " " + ihmcEtherCATIMU.getIMU0qy() + " " + ihmcEtherCATIMU.getIMU0qz());
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
