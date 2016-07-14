package us.ihmc.soem.examples;

import java.io.IOException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PeriodicParameters;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.soem.slaves.EK1100;
import us.ihmc.soem.slaves.EL3314;
import us.ihmc.soem.slaves.EL4134;
import us.ihmc.soem.wrapper.DistributedClockRealtimeThread;
import us.ihmc.soem.wrapper.Master;

public class DCExample extends DistributedClockRealtimeThread
{

   private static final int period = 1000000;
   private final Master master;
   private final EK1100 ek1100 = new EK1100(0, 0);
   private final EL3314 el3314 = new EL3314(601, 0);
   private final EL4134 el4134 = new EL4134(202, 0);

   private final SerialPortRTSPulseGenerator pulseGenerator = new SerialPortRTSPulseGenerator("/dev/ttyS7");

   private boolean signal = false;

   public DCExample() throws IOException
   {
      super("eth2", new PriorityParameters(PriorityParameters.getMaximumPriority()), new PeriodicParameters(new MonotonicTime(0, period)), 50000);
      master = getMaster();
      master.registerSlave(ek1100);
      master.registerSlave(el3314);
      master.registerSlave(el4134);
      master.enableDC(period);
      master.init();

   }

   @Override
   public void run()
   {
      int counter = 0;
      while (true)
      {
         if (waitForNextPeriodAndDoTransfer())
         {

            //        signal = !signal;
            //        pulseGenerator.setRTS(signal);
            //        el4134.setOut1(signal?Short.MAX_VALUE:Short.MIN_VALUE);

            if (counter++ % 100 == 0)
            {
               pulseGenerator.setRTS(true);
               el4134.setOut1(Short.MAX_VALUE);
            }
            else
            {
               pulseGenerator.setRTS(false);
               el4134.setOut1(Short.MIN_VALUE);
            }
         }
         else
         {
            System.out.println("DEADLINE MISSED");
         }
      }

   }

   public static void main(String[] args) throws IOException
   {
      DCExample ek1100example = new DCExample();
      ek1100example.start();
      ek1100example.join();
   }

}
