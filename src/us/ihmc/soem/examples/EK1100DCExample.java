package us.ihmc.soem.examples;

import java.io.IOException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.soem.slaves.EK1100;
import us.ihmc.soem.wrapper.Master;

public class EK1100DCExample implements Runnable
{

   private final Master master = new Master("enx94103eb91527");
   private final EK1100 ek1100 = new EK1100(0, 0);
   
   public EK1100DCExample() throws IOException
   {
      master.registerSlave(ek1100);
      master.enableDC();
      master.init();
      ek1100.configureDCSync0(true, 10000000, 0);
   }

   @Override
   public void run()
   {
      master.receive();
    master.send();

   }
   
   
   
   public static void main(String[] args) throws IOException
   {
      EK1100DCExample ek1100example = new EK1100DCExample();
      
      ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
      executor.scheduleAtFixedRate(ek1100example, 0, 10, TimeUnit.MILLISECONDS);
   }

}
