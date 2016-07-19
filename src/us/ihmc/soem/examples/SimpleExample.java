package us.ihmc.soem.examples;

import java.io.IOException;
import java.util.concurrent.locks.LockSupport;

import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.soem.slaves.beckhoff.EK1100;
import us.ihmc.soem.slaves.beckhoff.EL3314;
import us.ihmc.soem.slaves.beckhoff.EL4134;
import us.ihmc.soem.wrapper.EtherCATController;
import us.ihmc.soem.wrapper.Master;

public class SimpleExample extends Thread implements EtherCATController
{

   private final Master master;

   // Create slaves
   private final EK1100 ek1100 = new EK1100(0, 0); // Coupler
   private final EL3314 el3314 = new EL3314(601, 0); // Random slave that is plugged in our test bench
   private final EL4134 el4134 = new EL4134(202, 0); // Analog output

   private volatile boolean running = true;
   private boolean signal = false;

   public SimpleExample() throws IOException
   {
      master = new Master("eth2", this);
      // Register slaves to the master
      master.registerSlave(ek1100);
      master.registerSlave(el3314);
      master.registerSlave(el4134);
      master.init();
   }

   @Override
   public void run()
   {
      while(running)
      {
         master.send();
         master.receive();
         
         el4134.setOut1(signal ? Short.MAX_VALUE : Short.MIN_VALUE);
         signal = !signal;
         
         LockSupport.parkNanos(1000000);
      }
   }

   public static void main(String[] args) throws IOException
   {
      SimpleExample example = new SimpleExample();
      example.start();
   }

   @Override
   public void stopController()
   {
      running = false;
   }

}
