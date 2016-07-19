package us.ihmc.soem.examples;

import java.io.IOException;

import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.soem.slaves.elmo.ElmoTwitter;
import us.ihmc.soem.slaves.elmo.ElmoTwitterTCB;
import us.ihmc.soem.wrapper.DistributedClockRealtimeThread;
import us.ihmc.soem.wrapper.Master;

public class ElmoTwitterTCBExample extends DistributedClockRealtimeThread
{
   private final Master master;
   private final ElmoTwitter carrier = new ElmoTwitterTCB(0, 0);

   public ElmoTwitterTCBExample() throws IOException
   {
      super("eth2", new PriorityParameters(PriorityParameters.getMaximumPriority()), new MonotonicTime(0, 1000000), 100000);
      master = getMaster();
      master.registerSlave(carrier);
      master.init();
      // Test SDO reading
      byte receiveSDOLength = carrier.readSDOByte(0x1c12, 0x0);
      System.out.println("Number of elements in receive PDO: " + receiveSDOLength);
      for (int i = 0; i < receiveSDOLength; i++)
      {
         System.out.println("RxPDO assignment " + i + ": " + Integer.toHexString(carrier.readSDOUnsignedShort(0x1c12, i + 1)));
      }

   }

   public void run()
   {

      while (isRunning())
      {
         waitForNextPeriodAndDoTransfer();
         //      System.out.println(carrier.getPositionActualValue());
         //      System.out.println(carrier.getDigitalInputs());
         //      System.out.println(carrier.getVelocityActualValue());
         //      System.out.println(carrier.getStatus());
         //      System.out.println(carrier.getActualTorque());
         //      System.out.println(carrier.getPositionFollowingError());
         //      System.out.println(carrier.getActualAuxiliaryPosition());
         //      System.out.println(carrier.getElmoStatusRegister());
      }
   }

   public static void main(String[] args) throws IOException
   {
      ElmoTwitterTCBExample elmoTwitterTTSpecialCarrierExample = new ElmoTwitterTCBExample();
      elmoTwitterTTSpecialCarrierExample.start();
      elmoTwitterTTSpecialCarrierExample.join();
   }

}
