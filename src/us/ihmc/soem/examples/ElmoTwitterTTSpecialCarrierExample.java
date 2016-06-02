package us.ihmc.soem.examples;

import java.io.IOException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.soem.slaves.ElmoTwitterTTSpecialCarrier;
import us.ihmc.soem.wrapper.Master;

public class ElmoTwitterTTSpecialCarrierExample implements Runnable
{
   private final Master master = new Master("enx9cebe835ae57");
   private final ElmoTwitterTTSpecialCarrier carrier = new ElmoTwitterTTSpecialCarrier(master, 0, false, 1000000);
   
   
   public ElmoTwitterTTSpecialCarrierExample() throws IOException
   {
      master.registerSlave(carrier);
      master.init();
   }
   
   public void run()
   {
      master.receive();
      
//      System.out.println("-----");
      System.out.println(carrier.getPositionActualValue());
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
      ElmoTwitterTTSpecialCarrierExample elmoTwitterTTSpecialCarrierExample = new ElmoTwitterTTSpecialCarrierExample();
      
      ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
      executor.scheduleAtFixedRate(elmoTwitterTTSpecialCarrierExample, 0, 10, TimeUnit.MILLISECONDS);
   }
}
