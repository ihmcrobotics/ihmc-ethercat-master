package us.ihmc.etherCAT.examples;

import java.io.IOException;

import us.ihmc.etherCAT.master.EtherCATRealtimeThread;
import us.ihmc.etherCAT.slaves.elmo.ElmoTwitterTCB;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;

public class ElmoTwitterTCBExample extends EtherCATRealtimeThread
{
   private final ElmoTwitterTCB carrier = new ElmoTwitterTCB(0, 0);

   public ElmoTwitterTCBExample() throws IOException
   {
      super("eth2", new PriorityParameters(PriorityParameters.getMaximumPriority()), new MonotonicTime(0, 1000000), true, 100000);
      registerSlave(carrier);
      // Test SDO reading
      byte receiveSDOLength = carrier.readSDOByte(0x1c12, 0x0);
      System.out.println("Number of elements in receive PDO: " + receiveSDOLength);
      for (int i = 0; i < receiveSDOLength; i++)
      {
         System.out.println("RxPDO assignment " + i + ": " + Integer.toHexString(carrier.readSDOUnsignedShort(0x1c12, i + 1)));
      }

   }
   
   public static void main(String[] args) throws IOException
   {
      ElmoTwitterTCBExample elmoTwitterTTSpecialCarrierExample = new ElmoTwitterTCBExample();
      elmoTwitterTTSpecialCarrierExample.start();
      elmoTwitterTTSpecialCarrierExample.join();
   }

   @Override
   protected void deadlineMissed()
   {
      
   }

   @Override
   protected void doControl()
   {
      System.out.println(carrier.getPositionActualValue());
      System.out.println(carrier.getDigitalInputs());
      System.out.println(carrier.getVelocityActualValue());
      System.out.println(carrier.getStatus());
      System.out.println(carrier.getActualTorque());
      System.out.println(carrier.getPositionFollowingError());
      System.out.println(carrier.getActualAuxiliaryPosition());
      System.out.println(carrier.getElmoStatusRegister());
      
   }

   @Override
   protected void workingCounterMismatch(int expected, int actual)
   {
      carrier.disableDrive();

   }

   @Override
   protected void datagramLost()
   {
      // TODO Auto-generated method stub
      
   }

}
