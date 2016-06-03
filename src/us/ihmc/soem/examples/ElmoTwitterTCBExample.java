package us.ihmc.soem.examples;

import java.io.IOException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.soem.slaves.ElmoTwitterTCB;
import us.ihmc.soem.wrapper.Master;

public class ElmoTwitterTCBExample implements Runnable
{
   private final Master master = new Master("enx9cebe835ae57");
   private final ElmoTwitterTCB carrier = new ElmoTwitterTCB(master, 0, false, 1000000);
   
   
   public ElmoTwitterTCBExample() throws IOException
   {
      master.registerSlave(carrier);
      master.configureDC(10000000);
      master.init();
      
      
      carrier.configureDCSync0(true, 10000000, 0);
      
      // Test SDO reading
      byte receiveSDOLength = carrier.readSDOByte(0x1c12, 0x0);
      System.out.println("Number of elements in receive PDO: " + receiveSDOLength);
      for(int i = 0; i < receiveSDOLength; i++)
      {
         System.out.println("RxPDO assignment " + i + ": " + Integer.toHexString(carrier.readSDOUnsignedShort(0x1c12, i + 1)));
      }
      
   }
   
   public void run()
   {
      master.receive();
      
//      System.out.println(carrier.getPositionActualValue());
//      System.out.println(carrier.getDigitalInputs());
//      System.out.println(carrier.getVelocityActualValue());
//      System.out.println(carrier.getStatus());
//      System.out.println(carrier.getActualTorque());
//      System.out.println(carrier.getPositionFollowingError());
//      System.out.println(carrier.getActualAuxiliaryPosition());
//      System.out.println(carrier.getElmoStatusRegister());
      
      master.send();
   }
   
   public static void main(String[] args) throws IOException
   {
      ElmoTwitterTCBExample elmoTwitterTTSpecialCarrierExample = new ElmoTwitterTCBExample();
      
      ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
      executor.scheduleAtFixedRate(elmoTwitterTTSpecialCarrierExample, 0, 10, TimeUnit.MILLISECONDS);
   }
}
