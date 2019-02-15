package us.ihmc.etherCAT.master;

import java.io.IOException;
import java.util.List;

import us.ihmc.process.LinuxProcess;
import us.ihmc.process.Scheduler;
import us.ihmc.process.SchedulerAlgorithm;

/**
 * Simple class to increase the priority of an Ethernet IRQ thread on a Linux host. This could drastically decrease jitter.
 * 
 * @author jesper
 *
 */
public class EthernetIRQPriority
{
   /**
    * Increase priority of the irq threads for a network interface on a Linux system, with the thread name matching irq/d+-[ifname].
    * 
    * @param ifname
    */
   public static void increaseEthernetIRQPriority(String ifname)
   {
      try
      {
         List<LinuxProcess> ethernetInterruptThreads = LinuxProcess.getProcessesByPattern("irq/\\d+-" + ifname + ".*");
         for(LinuxProcess ethernetInterruptThread : ethernetInterruptThreads)
         {
            Scheduler.setScheduler(ethernetInterruptThread, SchedulerAlgorithm.SCHED_FIFO, 90);
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}
