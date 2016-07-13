package us.ihmc.soem.wrapper;

import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PeriodicParameters;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;

public class DistributedClockRealtimeThread extends RealtimeThread
{
   private final Master master;
   private long dcControlIntegral = 0;
   private long syncOffset = 0;

   private boolean initialized = false;
   private final MonotonicTime triggerTime = new MonotonicTime();
   private final MonotonicTime update = new MonotonicTime();

   private final long cycleTimeInNs;

   public DistributedClockRealtimeThread(String iface, PriorityParameters priorityParameters, PeriodicParameters periodicParameters, long syncOffset)
   {
      this(new Master(iface), priorityParameters, periodicParameters, syncOffset);
   }

   public DistributedClockRealtimeThread(Master master, PriorityParameters priorityParameters, PeriodicParameters periodicParameters, long syncOffset)
   {
      super(priorityParameters, periodicParameters);

      if (periodicParameters == null)
      {
         throw new RuntimeException("This class requires a period");
      }

      this.cycleTimeInNs = periodicParameters.getPeriod().asNanoseconds();
      this.syncOffset = syncOffset;
      this.master = master;
      master.enableDC();
   }

   public Master getMaster()
   {
      return master;
   }

   public void waitForNextPeriodAndDoTransfer()
   {
      waitForNextPeriod();
      master.send();
      master.receive();
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

   
   int i = 0;
   @Override
   public final long waitForNextPeriod()
   {
      if (!initialized)
      {
         triggerTime.set(0, (getCurrentMonotonicClockTime() / cycleTimeInNs) * cycleTimeInNs + cycleTimeInNs);
         initialized = true;
      }

      long offset = calculateDCOffsetTime(syncOffset);
      long update = cycleTimeInNs + offset;
      this.update.set(0, update);
      triggerTime.add(this.update);
      
      return super.waitUntil(triggerTime);
//      return super.waitForNextPeriod();
   }

}
