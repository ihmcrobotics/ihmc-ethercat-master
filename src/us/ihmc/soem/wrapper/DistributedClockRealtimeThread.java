package us.ihmc.soem.wrapper;

import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PeriodicParameters;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;

/**
 * Thread that is synchronized to the DC Master clock
 * 
 * @author Jesper Smith
 *
 */
public abstract class DistributedClockRealtimeThread extends RealtimeThread implements EtherCATController
{
   private final Master master;
   private long dcControlIntegral = 0;
   private long syncOffset = 0;

   private boolean initialized = false;
   private final MonotonicTime initialTriggerTime = new MonotonicTime();

   private final long cycleTimeInNs;
   private volatile boolean running = true;
   
   
   /**
    * Create new Thread synchornized to the DC Master Clock
    * 
    * @param master EtherCAT Master
    * @param priorityParameters Desired priority
    * @param period Desired period
    * @param syncOffset Waiting time between the DC Master Clock and calling master.send(). Recommended to be between 50000ns and 100000ns depending on system, CPU load and loop times. 
    */
   public DistributedClockRealtimeThread(String iface, PriorityParameters priorityParameters, MonotonicTime period, long syncOffset)
   {
      super(priorityParameters, new PeriodicParameters(period));


      this.cycleTimeInNs = period.asNanoseconds();
      this.syncOffset = syncOffset;
      this.master = new Master(iface, this);
      master.enableDC(period.asNanoseconds());
   }

   /**
    * @return the EtherCAT master
    */
   public Master getMaster()
   {
      return master;
   }
   

   /**
    * Call every tick to wait for the next trigger time followed by the EtherCAT transaction.
    * 
    * @return true if deadline was met and the EtherCAT transaction made
    */
   public boolean waitForNextPeriodAndDoTransfer()
   {
      if(waitForNextPeriod() > 0)
      {
         master.send();
         master.receive();
         return true;
      }
      else
      {
         return false;
      }
   }
   
   /* PI calculation to get linux time synced to DC time */

   /**
    * Simple PI controller to be used to synchronize the control loop with the 
    * Distributed Clocks feature of EtherCAT.   
    * 
    * @param syncOffset Offset from the start of the DC sync pulse.
    * 
    * @return Offset in NS to add to the current tick duration to synchronize the clocks
    */
   public long calculateDCOffsetTime(long syncOffset)
   {
      long reftime = master.getDCTime();

      /* set linux sync point 50us later than DC sync, just as example */
      long delta = (reftime - syncOffset) % cycleTimeInNs;
      if (delta > (cycleTimeInNs / 2))
      {
         delta = delta - cycleTimeInNs;
      }
      if (delta > 0)
      {
         dcControlIntegral++;
      }
      if (delta < 0)
      {
         dcControlIntegral--;
      }
      return -(delta / 100) - (dcControlIntegral / 20);
   }

   
   /**
    * Wait for next trigger period. It is recommended to use waitForNextPeriodAndDoTransfer()
    * 
    */
   @Override
   public final long waitForNextPeriod()
   {
      if (!initialized)
      {
         initialTriggerTime.set(0, (getCurrentMonotonicClockTime() / cycleTimeInNs) * cycleTimeInNs + cycleTimeInNs); // Round trigger time to cycletime
         setNextPeriod(initialTriggerTime);
         initialized = true;
      }

      long offset = calculateDCOffsetTime(syncOffset);
      
      return super.waitForNextPeriod(offset);
   }

   /**
    * 
    * Check to see if the EtherCAT controller should continue execution. Use this as your test for your while loop.
    * 
    * @return true if this thread should continue execution
    */
   public boolean isRunning()
   {
      return running;
   }
   
   @Override
   public void stopController()
   {
      running = false;
   }

}
