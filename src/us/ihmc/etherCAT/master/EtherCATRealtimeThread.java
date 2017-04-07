package us.ihmc.etherCAT.master;

import java.io.IOException;
import java.util.List;

import us.ihmc.affinity.Processor;
import us.ihmc.etherCAT.master.Slave.State;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PeriodicParameters;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.soem.generated.soem;

/**
 * Thread that is synchronized to the DC Master clock or Free running.
 * 
 * This is the main class to interact with the Master. It takes care of 
 * all the details of the EtherCAT transmission and the usage of Distributed Clocks.
 * 
 * It also takes care of taring down EtherCAT slaves properly on shutdown.
 * 
 * @author Jesper Smith
 *
 */
public abstract class EtherCATRealtimeThread implements MasterInterface
{
   
   private final RealtimeThread realtimeThread;
   private final Master master;
   private boolean enableDC;
   
   private long dcControlIntegral = 0;
   private long syncOffset = 0;

   private final long cycleTimeInNs;
   private volatile boolean running = true;
   
   private long currentCycleTimestamp = 0;
   private long etherCATTransactionTime = 0;
   private long etherCATStateMachineTime = 0;
   private long lastCycleDuration = 0;
   private long idleTime = 0;
   private long startTimeFreeRun = 0;
   private long dcOffsetError = 0;
   
   private long cycleStartTime = 0;
   
   private long dcTime = 0;
   
   private boolean inOP = false;
   
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
      this.realtimeThread = new RealtimeThread(priorityParameters, new PeriodicParameters(period), new Runnable()
      {
         @Override
         public void run()
         {
            EtherCATRealtimeThread.this.run();
         }
      });


      this.cycleTimeInNs = period.asNanoseconds();
      this.syncOffset = syncOffset;
      this.master = new Master(iface);
      this.enableDC = enableDC;
      if(enableDC)
      {
         master.enableDC(period.asNanoseconds());
         master.disableRecovery();// Disable recovery mode because it doesn't work under DC at the moment
      }
      

      Runtime.getRuntime().addShutdownHook(new Thread()
      {
         public void run()
         {
            EtherCATRealtimeThread.this.stopController();
            EtherCATRealtimeThread.this.join();
         }
      });
   }

   /**
    * Starts the EtherCAT controller.
    */
   public void start()
   {
      realtimeThread.start();
   }
   
   /** 
    * Delegate function to be able to switch to non-realtime threads in the future
    * 
    * @return Current monotonic clock time
    */
   private long getCurrentMonotonicClockTime()
   {
      return RealtimeThread.getCurrentMonotonicClockTime();
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
      return currentCycleTimestamp;
   }
   
   /** 
    * Internal function. Calculated the timestamp for the current cycle
    * @return timestamp for the current cycle.
    */
   private long calculateCurrentCycleTimestamp()
   {
      if(enableDC)
      {
         // Rounding the DC time down to the cycle time gives the previous sync0 time.
         return (dcTime / cycleTimeInNs) * cycleTimeInNs;
      }
      else
      {
         return getCurrentMonotonicClockTime();
      }
   }
   
   


   /**
    * Returns the timestamp at the start of the cyclic execution
    * 
    * If enableDC = true, the timestamp will be the sync0 time on the DC Master Clock taken during master.init()
    * If enableDC = false, the timestamp will be monontonic time taken from the computer running this code at the end of master.init()
    * 
    */
   public long getInitTimestamp()
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
    * @return Time spent parked waiting for next execution. Does not include EtherCAT transaction time.
    */
   public long getIdleTime()
   {
      return idleTime;
   }
   
   /**
    * 
    * @return Time spent doing the EtherCAT transaction. 
    */
   public long getEtherCATTransactionTime()
   {
      return etherCATTransactionTime;
   }
   
   /**
    * 
    * @return The error between the desired and actual DC receive time on slave 0.  
    */
   public long getDCOffsetError()
   {
      return dcOffsetError;
   }
   
   /**
    * Main loop.
    */
   private final void run()
   {
      try
      {
         master.init();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      
      this.enableDC = master.getDCEnabled();
      
      if(!enableDC)
      {
         startTimeFreeRun = getCurrentMonotonicClockTime();
      }
      
      while(running)
      {
         if(waitForNextPeriodAndDoTransfer() && inOP)
         {            
            currentCycleTimestamp = calculateCurrentCycleTimestamp();  
            doControl();
         }
         else if (!inOP)
         {
            if(master.getState() == State.OP)
            {
               inOP = true;
            }
         }
         
         doReporting();
         
         
      }
      
      boolean allSlavesShutdown = false;
      while(!allSlavesShutdown)
      {
         if(waitForNextPeriodAndDoTransfer())
         {
            allSlavesShutdown = master.shutdownSlaves();
         }
         else
         {
            deadlineMissed();
         }
      }
      
      master.shutdown();
   }

   
   /**
    * Call every tick to wait for the next trigger time followed by the EtherCAT transaction.
    * 
    * @return true if deadline was met and the EtherCAT transaction was successful
    */
   private boolean waitForNextPeriodAndDoTransfer()
   {
      idleTime = waitForNextPeriodInternal();
      long currentTime = getCurrentMonotonicClockTime();
      lastCycleDuration =  currentTime - cycleStartTime;
      cycleStartTime = currentTime;
      
      if(idleTime > 0)
      {
         long startTime = getCurrentMonotonicClockTime();
         master.send();
         int wkc = master.receive();
         
         if(wkc == soem.EC_NOFRAME)
         {
            datagramLost();
            etherCATTransactionTime = getCurrentMonotonicClockTime() - startTime;
            return false;
         }
         
         dcTime = master.getDCTime();
         if(inOP && wkc != master.getExpectedWorkingCounter())
         {
            workingCounterMismatch(master.getExpectedWorkingCounter(), wkc);
         }
         
	      long ethercatStateMachineStartTime = getCurrentMonotonicClockTime();
         master.doEtherCATStateControl();
	      long endTime = getCurrentMonotonicClockTime();
         etherCATTransactionTime = endTime - startTime;
         etherCATStateMachineTime = endTime - ethercatStateMachineStartTime;
         
         return true;
      }
      else
      {
         deadlineMissed();
         return false;
      }
   }
   

   /**
    * Do an EtherCAT send/receive cycle.
    * 
    * Use this to decrease latency by sending two packets per control cycle. This will allow sending new setpoints before the next pulse happens.
    * 
    * @param extraTimeHeadroomInNs Extra time available on top of the last etherCAT transaction time and sync offset
    *
    * @return true if there is enough time to do an EtherCAT transaction and the transaction was successful
    * 
    */
   public boolean doSecondaryTransfer(long extraTimeHeadroomInNs) 
   {
      long currentTime = getCurrentMonotonicClockTime();
      if(((currentTime - cycleStartTime) + syncOffset + etherCATTransactionTime + extraTimeHeadroomInNs) < cycleTimeInNs)
      {
         master.send();
         int wkc = master.receiveSimple();
         if(wkc == soem.EC_NOFRAME)
         {
            datagramLost();
            return false;
         }
         else if(wkc != master.getExpectedWorkingCounter())
         {
            workingCounterMismatch(master.getExpectedWorkingCounter(), wkc);
            return false;
         }
         else
         {
            return true;
         }
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
   private long calculateDCOffsetTime(long syncOffset)
   {
      long reftime = dcTime;

      dcOffsetError = (reftime - syncOffset) % cycleTimeInNs;
      if (dcOffsetError > (cycleTimeInNs / 2))
      {
         dcOffsetError = dcOffsetError - cycleTimeInNs;
      }
      if (dcOffsetError > 0)
      {
         dcControlIntegral++;
      }
      if (dcOffsetError < 0)
      {
         dcControlIntegral--;
      }
      return -(dcOffsetError / 100) - (dcControlIntegral / 20);
   }

   /**
    * Internal function. Delegate to wait for next period, depending on DC enabled/disabled. 
    * 
    * @return time waited
    */
   private final long waitForNextPeriodInternal()
   {
      if(enableDC)
      {
         long offset = calculateDCOffsetTime(syncOffset);
         return realtimeThread.waitForNextPeriod(offset);         
      }
      else
      {
         return realtimeThread.waitForNextPeriod();
      }
   }

   /**
    * Calling this function stops the EtherCAT thread at the next iteration. This function is safe to be called from any thread.
    */
   public final void stopController()
   {
      running = false;
   }

   /**
    * Wait till the EtherCAT thread has finished executing.
    */
   public void join()
   {
      realtimeThread.join();
   }
   
   
   /**
    * @see us.ihmc.etherCAT.master.Master#setEtherCATStatusCallback(us.ihmc.etherCAT.master.EtherCATStatusCallback)
    */
   public void setEtherCATStatusCallback(EtherCATStatusCallback callback)
   {
      master.setEtherCATStatusCallback(callback);
   }

   /**
    * @see us.ihmc.etherCAT.master.Master#enableTrace()
    */
   public void enableTrace()
   {
      master.enableTrace();
   }

   /**
    * @see us.ihmc.etherCAT.master.Master#registerSDO(us.ihmc.etherCAT.master.SDO)
    */
   @Override
   public void registerSDO(SDO sdo)
   {
      master.registerSDO(sdo);
   }

   /**
    * @see us.ihmc.etherCAT.master.Master#registerSlave(us.ihmc.etherCAT.master.Slave)
    */
   @Override
   public void registerSlave(Slave slave)
   {
      master.registerSlave(slave);
   }

   /**
    * @see us.ihmc.etherCAT.master.Master#getJitterEstimate()
    */
   @Override
   public long getJitterEstimate()
   {
      return master.getJitterEstimate();
   }

   /**
    * @see us.ihmc.etherCAT.master.Master#setMaximumExecutionJitter(long)
    */
   @Override
   public void setMaximumExecutionJitter(long jitterInNanoseconds)
   {
      master.setMaximumExecutionJitter(jitterInNanoseconds);
   }
   
   /**
    * @see us.ihmc.etherCAT.master.Master#setRequireAllSlaves(boolean)
    */
   public void setRequireAllSlaves(boolean requireAllSlaves)
   {
      master.setRequireAllSlaves(requireAllSlaves);
   }
   
   /**
    * The measured cycle time of the previous control tick. Should be equal to the desired period with a small amount of jitter.
    * 
    * @return Duration of the complete control cycle, including time spent waiting for the next cycle.
    */
   public long getLastCycleDuration()
   {
      return lastCycleDuration;
   }

   /**
    * The time the ethercat state machine took. This should be close to zero during normal operation
    *
    * @return time ethercat state machine took
    */
   public long getEtherCATStateMachineTime()
   {
      return etherCATStateMachineTime;
   }

   /**
    * 
    * @see us.ihmc.realtime.RealtimeThread#setAffinity(Processor...)
    */
   public void setAffinity(Processor... processors)
   {
      realtimeThread.setAffinity(processors);
   }


   /**
    * Returns a new unmoddifiable list with all slaves registered with the master.
    *
    * @return all slaves registered with the master
    */
   @Override
   public List<Slave> getSlaves()
   {
      return master.getSlaves();
   }


   /**
    * Callback to notify controller that there was a difference in working counter
    * 
    * This gets called when the expected working counter and actual working counter differ. 
    * It is recommended to go to a safe state when this function gets called.
    * 
    * This function gets called before the state of the slaves is updated. All slaves will probably be
    * in OP mode till the EtherCAT householder thread checks the state. This can take 10ms or more.
    * 
    */
   protected abstract void workingCounterMismatch(int expected, int actual);
   
   /**
    * Callback to notify controller of missed deadline
    */
   protected abstract void deadlineMissed();
   
   /**
    * Callback called cyclically to do the control loop. 
    * 
    * Will not get called till all actuators are online. Will also not get called when a deadline is missed or a datagram got lost.
    */
   protected abstract void doControl();
   
   /**
    * Callback called cyclically to do reporting and logging.
    * 
    * Will always get called, even when the controller is not yet online or a deadline is missed. Make sure this function returns quickly.
    */
   protected abstract void doReporting();
   
   /**
    * The receive function of the master timed out and no packet was received.
    * 
    * This means a packet got corrupted or dropped on the slave network and is generally bad news.
    * 
    */
   protected abstract void datagramLost();



}
