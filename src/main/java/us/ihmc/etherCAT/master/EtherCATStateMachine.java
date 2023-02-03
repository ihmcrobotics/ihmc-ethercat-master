package us.ihmc.etherCAT.master;

import java.util.ArrayList;

import us.ihmc.etherCAT.master.EtherCATStatusCallback.TRACE_EVENT;
import us.ihmc.soem.generated.ec_slavet;
import us.ihmc.soem.generated.ec_state;
import us.ihmc.soem.generated.soem;

/**
 * Statemachine to do the state control for EtherCAT slaves.
 * 
 * This will configure the slaves to go in OP when the master reached stable execution as well as deal with slaves that got into an error mode
 * 
 * @author Jesper Smith
 *
 */
class EtherCATStateMachine
{
   public static final long MINIMUM_JITTER_SAMPLES = 1000;
   
   public static boolean DEBUG = false;

   private final Master master;
   private Slave[] slaves;

   private final ReadState readState = new ReadState();
   private final WaitForMasterState waitForMasterState = new WaitForMasterState();
   private final CheckForLostSlavesState checkForLostSlavesState = new CheckForLostSlavesState();
   private final FinalState finalState = new FinalState();
   private EtherCATState slaveState;

   private long startTime = -1;
   private long runTime = 0;
   
   private int currentSDO = 0;
   
   private EtherCATState currentState;
   
   private boolean recoveryDisabled = false;

   EtherCATStateMachine(Master master)
   {
      this.master = master;
   }
   
   void setSlaves(Slave[] slaves)
   {
      this.slaves = slaves;
      EtherCATState next = readState;
      for (int i = slaves.length - 1; i >= 0; i--)
      {
         SlaveState state = new SlaveState(slaves[i], next);
         next = state;
      }

      this.slaveState = next;
      

      currentState = readState;
   }
   
   private void info(String msg)
   {
      System.out.println("[EtherCATStateMachine] " + msg);
   }
   
   void disableRecovery()
   {
      recoveryDisabled = true;
   }

   void doStateControl()
   {
      if (startTime < 0)
      {
         startTime = System.nanoTime();
      }
      runTime = System.nanoTime() - startTime;
      
      if(DEBUG)
      {
         info(" - " + currentState.getClass().getSimpleName());
      }
      
      currentState = currentState.next();
   }
   
   private int updateSlaveStates()
   {
      int state = soem.ecx_readstate(master.getContext());
      for (int i = 0; i < slaves.length; i++)
      {
         slaves[i].updateEtherCATState();
      }
      return state;
   }

   private void doSDOTransfer()
   {
      ArrayList<SDO> sdos = master.getSDOs();
      
      if(sdos.size() > 0)
      {
         for(int c = 0; c < sdos.size(); c++)
         {
            if(++currentSDO >= sdos.size())
            {
               currentSDO = 0;
            }
            
            if(sdos.get(currentSDO).update())
            {
               return;
            }
            
         }
         
      }
   }
   
   
   private interface EtherCATState
   {
      EtherCATState next();
   }

   private class ReadState implements EtherCATState
   {
      @Override
      public EtherCATState next()
      {
         int state = updateSlaveStates();
         if (state == ec_state.EC_STATE_OPERATIONAL.swigValue())
         {
            if(recoveryDisabled)
            {
               return finalState;
            }
            else
            {
               return checkForLostSlavesState;
            }
         }
         else
         {
            return waitForMasterState;
         }
      }
   }

   /**
    * State to wait for the master to attain a stable, minimal jitter rate.
    *
    */
   private class WaitForMasterState implements EtherCATState
   {
      @Override
      public EtherCATState next()
      {
         long jitterEstimate = master.getJitterEstimate();

         if (!master.getDCEnabled())
         {
            return slaveState;
         }
         else if (master.getJitterSamples() < MINIMUM_JITTER_SAMPLES)
         {
            return this;
         }
         else if (jitterEstimate == 0 || jitterEstimate > master.getMaximumExecutionJitter())
         {
            master.getEtherCATStatusCallback().reportMasterThreadStableRateTime(runTime, jitterEstimate);
            return this;
         }
         else
         {
            return slaveState;
         }

      }
   }

   /**
    * Run all individual state slave machines
    *
    */
   private class SlaveState implements EtherCATState
   {
      private final Slave slave;
      private final EtherCATState next;

      private SlaveState(Slave slave, EtherCATState next)
      {
         this.slave = slave;
         this.next = next;
      }

      @Override
      public EtherCATState next()
      {
         slave.doEtherCATStateControl(runTime);
         return next;
      }

   }

   /**
    * State to check for lost slaves. If slaves are lost, read the state and reconfigure
    * slaves to Operational mode
    * 
    */
   private class CheckForLostSlavesState implements EtherCATState
   {

      @Override
      public EtherCATState next()
      {
         if (master.getExpectedWorkingCounter() != master.getActualWorkingCounter())
         {
            return readState;
         }
         else
         {
            doSDOTransfer();
            return this;
         }
      }

   }
   
   /**
    * State that checks for lost slaves, but will not take action to reconfigure them.
    * 
    * Used when recoveryDisabled = true
    *
    */
   private class FinalState implements EtherCATState
   {
      long previousActualWorkingCounter = 0;
      
      @Override
      public EtherCATState next()
      {   
         int wkc = master.getActualWorkingCounter(); 
         if (wkc != master.getExpectedWorkingCounter())
         {
            if(previousActualWorkingCounter != wkc)
            {
               updateSlaveStates();
            }
            else
            {
               doSDOTransfer();
            }
         }
         else
         {
            doSDOTransfer();            
         }
         
         previousActualWorkingCounter = wkc;
         
         return this;
      }
   }

   void shutDown()
   {
      master.getEtherCATStatusCallback().trace(TRACE_EVENT.SWITCH_PREOP);
      ec_slavet allSlaves = soem.ecx_slave(master.getContext(), 0);
      allSlaves.setState(ec_state.EC_STATE_PRE_OP.swigValue());
      soem.ecx_writestate(master.getContext(), 0);

      master.getEtherCATStatusCallback().trace(TRACE_EVENT.CLEANUP_SLAVES);

      for (int i = 0; i < slaves.length; i++)
      {
         slaves[i].cleanup();
      }

   }

}
