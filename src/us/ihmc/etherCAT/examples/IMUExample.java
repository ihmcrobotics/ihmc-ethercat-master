package us.ihmc.etherCAT.examples;

import us.ihmc.etherCAT.master.EtherCATRealtimeThread;
import us.ihmc.etherCAT.slaves.beckhoff.EK1100;
import us.ihmc.etherCAT.slaves.beckhoff.EL3314;
import us.ihmc.etherCAT.slaves.beckhoff.EL4134;
import us.ihmc.etherCAT.slaves.ihmc.IHMCEtherCATIMU;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;

public class IMUExample extends EtherCATRealtimeThread
{
   private final IHMCEtherCATIMU ihmcEtherCATIMU = new IHMCEtherCATIMU(3, 0);
   private final EK1100 ek1100 = new EK1100(101, 0); // Coupler
   private final EL3314 el3314 = new EL3314(201, 0); // Random slave that is plugged in our test bench
   private final EL4134 el4134 = new EL4134(601, 0); // Analog output
   private int counter = 0;
   
   public IMUExample()
   {
      super("p3p1", PriorityParameters.MAXIMUM_PRIORITY, new MonotonicTime(0, 1000000), true, 100000);
      registerSlave(ihmcEtherCATIMU);
      registerSlave(ek1100);
      registerSlave(el3314);
      registerSlave(el4134);
      enableTrace();
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
      System.out.println("DATAGRAM LOST");
   }
}
