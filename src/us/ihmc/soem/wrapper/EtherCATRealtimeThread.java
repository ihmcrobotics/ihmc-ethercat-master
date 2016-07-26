package us.ihmc.soem.wrapper;

import java.io.IOException;

import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PeriodicParameters;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;

/**
 * Thread that is synchronized to the DC Master clock or Free running
 * 
 * @author Jesper Smith
 *
 */
public abstract class EtherCATRealtimeThread extends RealtimeThread implements EtherCATController
{
   private final Master master;
   private final boolean enableDC;
   
   private long dcControlIntegral = 0;
   private long syncOffset = 0;

   private final long cycleTimeInNs;
   private volatile boolean running = true;
   
   private long startTimeFreeRun = 0;
   
   
   /**
    * Create new Thread that is free running with respect to the slaves
    * 
    * @param iface Network interface that is connected to the EtherCAT network
    * @param priorityParameters Desired priority
    * @param period Desired period
    * @param enableDC false
    */
   public EtherCATRealtimeThread(String iface, PriorityParameters priorityParameters, MonotonicTime period, boolean enableDC)
   {
      this(iface, priorityParameters, period, false, -1);
      
      if(enableDC)
      {
         throw new RuntimeException("Please  provide a syncOffset to enable Distributed Clocks");
      }
   }
   
   /**
    * Create new Thread for EtherCAT communication.  
    * 
    * @param iface Network interface that is connected to the EtherCAT network
    * @param priorityParameters Desired priority
    * @param period Desired period
    * @param enableDC Enable Distributed clocks and synchronize this thread to the EtherCAT Master Clock 
    * @param syncOffset Waiting time between the DC Master Clock and calling master.send(). Recommended to be between 50000ns and 100000ns depending on system, CPU load and loop times.
    * 
    */
   public EtherCATRealtimeThread(String iface, PriorityParameters priorityParameters, MonotonicTime period, boolean enableDC, long syncOffset)
   {
      super(priorityParameters, new PeriodicParameters(period));


      this.cycleTimeInNs = period.asNanoseconds();
      this.syncOffset = syncOffset;
      this.master = new Master(iface, this);
      this.enableDC = enableDC;
      if(enableDC)
      {
         master.enableDC(period.asNanoseconds());
      }
   }

   
   /**
    * Returns the current cycle timestamp
    * 
    * If enableDC = true, the timestamp will be the sync0 time on the DC Master Clock
    * If enableDC = false, the timestamp will be monontonic time taken from the computer running this code
    * 
    * @return
    */
   public long getCurrentCycleTimestamp()
   {
      if(enableDC)
      {
         // Rounding the DC time down to the cycle time gives the previous sync0 time.
         return (master.getDCTime() / cycleTimeInNs) * cycleTimeInNs;
      }
      else
      {
         return getCurrentMonotonicClockTime();
      }
   }
   
   
   /**
    * Returns the timestamp at start
    * 
    * If enableDC = true, the timestamp will be the sync0 time on the DC Master Clock taken during init()
    * If enableDC = false, the timestamp will be monontonic time taken from the computer running this code at the end of init()
    * 
    */
   public long getStartDCTime()
   {
      if(enableDC)
      {
         return (master.getStartDCTime() / cycleTimeInNs) * cycleTimeInNs;         
      }
      else
      {
         return startTimeFreeRun;
      }
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
      if(enableDC)
      {
         long offset = calculateDCOffsetTime(syncOffset);
         return super.waitForNextPeriod(offset);         
      }
      else
      {
         return super.waitForNextPeriod();
      }
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

   /**
    * @see us.ihmc.soem.wrapper.Master#setEtherCATStatusCallback(us.ihmc.soem.wrapper.EtherCATStatusCallback)
    */
   public void setEtherCATStatusCallback(EtherCATStatusCallback callback)
   {
      master.setEtherCATStatusCallback(callback);
   }

   /**
    * @see us.ihmc.soem.wrapper.Master#enableTrace()
    */
   public void enableTrace()
   {
      master.enableTrace();
   }

   /**
    * @see us.ihmc.soem.wrapper.Master#registerSDO(us.ihmc.soem.wrapper.SDO)
    */
   public void registerSDO(SDO sdo)
   {
      master.registerSDO(sdo);
   }

   /**
    * @see us.ihmc.soem.wrapper.Master#init()
    */
   public void init() throws IOException
   {
      master.init();
      if(!enableDC)
      {
         startTimeFreeRun = getCurrentMonotonicClockTime();
      }
   }

   /**
    * 
    * @see us.ihmc.soem.wrapper.Master#shutdown()
    */
   public void shutdown()
   {
      master.shutdown();
   }

   /**
    * @see us.ihmc.soem.wrapper.Master#registerSlave(us.ihmc.soem.wrapper.Slave)
    */
   public void registerSlave(Slave slave)
   {
      master.registerSlave(slave);
   }

   /**
    * @see us.ihmc.soem.wrapper.Master#getJitterEstimate()
    */
   public long getJitterEstimate()
   {
      return master.getJitterEstimate();
   }

   /**
    * @see us.ihmc.soem.wrapper.Master#setMaximumExecutionJitter(long)
    */
   public void setMaximumExecutionJitter(long jitterInNanoseconds)
   {
      master.setMaximumExecutionJitter(jitterInNanoseconds);
   }


   
   

}
