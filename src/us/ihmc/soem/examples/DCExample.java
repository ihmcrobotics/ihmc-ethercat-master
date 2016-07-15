package us.ihmc.soem.examples;

import java.io.IOException;

import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.soem.slaves.EK1100;
import us.ihmc.soem.slaves.EL3314;
import us.ihmc.soem.slaves.EL4134;
import us.ihmc.soem.wrapper.DistributedClockRealtimeThread;
import us.ihmc.soem.wrapper.Master;

/**
 * Simple example to show the usage of Distributed Clocks. 
 * 
 * This code generates pulses on the RTS line of a serial port, and outputs the same pulse to the EL4134.
 * It is expected that the pulses are ~2ms different and the EL4134 pulse is 1ms exactly in length. There will
 * be some jitter on the serial pulse width.  
 * 
 * @author Jesper Smith
 *
 */
public class DCExample extends DistributedClockRealtimeThread
{
   public static final boolean SWITCH_SIGNAL_EVERY_TICK = true;

   private static final int period = 1000000;
   private final Master master;

   // Create slaves
   private final EK1100 ek1100 = new EK1100(0, 0); // Coupler
   private final EL3314 el3314 = new EL3314(601, 0); // Random slave that is plugged in our test bench
   private final EL4134 el4134 = new EL4134(202, 0); // Analog output


   private final SerialPortRTSPulseGenerator pulseGenerator = new SerialPortRTSPulseGenerator("/dev/ttyS7");

   
   private boolean signal = false;

   public DCExample() throws IOException
   {
      super("eth2", PriorityParameters.getRelativePriority(50), new MonotonicTime(0, period), 50000);

      master = getMaster();

      // Register slaves to the master
      master.registerSlave(ek1100);
      master.registerSlave(el3314);
      master.registerSlave(el4134);


   }

   @Override
   public void run()
   {

      int counter = 0;
      // Initialize master
      try
      {
         master.init();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      
      while (isRunning())
      {
         // Wait for the next period and do the transfer. If deadline is missed, skip calculations
         if (waitForNextPeriodAndDoTransfer())
         {

            if (SWITCH_SIGNAL_EVERY_TICK)
            {
               signal = !signal;
               pulseGenerator.setRTS(signal);
               el4134.setOut1(signal ? Short.MAX_VALUE : Short.MIN_VALUE);
            }
            else
            {
               // Generate a signal on both the serial RTS line of the computer and the analog output of the EL4134
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
