package us.ihmc.soem.wrapper;

import us.ihmc.realtime.PeriodicParameters;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;

public class DistributedClockRealtimeThread extends RealtimeThread
{
   private final Master master;
   private long dcCycleTime = 0;
   private long dcControlIntegral = 0;

   
   public DistributedClockRealtimeThread(String iface, PriorityParameters priorityParameters, PeriodicParameters periodicParameters)
   {
      this(new Master(iface), priorityParameters, periodicParameters);
   }
   
   public DistributedClockRealtimeThread(Master master, PriorityParameters priorityParameters, PeriodicParameters periodicParameters)
   {
      super(priorityParameters, periodicParameters);
      
      if(periodicParameters == null)
      {
         throw new RuntimeException("This class requires a period");
      }
      
      this.master = master;
      master.enableDC();
   }
   
   public Master getMaster()
   {
      return master;
   }
   
   /* PI calculation to get linux time synced to DC time */
   
   /**
    * Simple PI controller to be used to synchronize the control loop with the 
    * Distributed Clocks feature of EtherCAT.   
    * 
    * @param clockTime 
    * @param syncOffset Offset from the start of the DC sync pulse.
    * 
    * @return Offset in NS to add to the current tick duration to synchronize the clocks
    */
   public long calculateDCOffsetTime(long clockTime, long syncOffset)
   {
         long reftime = master.getDCTime();
         
         /* set linux sync point 50us later than DC sync, just as example */
         long delta = (reftime - syncOffset) % dcCycleTime;
         if(delta> (dcCycleTime /2)) { delta= delta - dcCycleTime; }
         if(delta>0){ dcControlIntegral++; }
         if(delta<0){ dcControlIntegral--; }
         return -(delta / 100) - (dcControlIntegral /20);
   }
   
   @Override
   public final long waitForNextPeriod()
   {
            
      return super.waitForNextPeriod();
   }
   
}
