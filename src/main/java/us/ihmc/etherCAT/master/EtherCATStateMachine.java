package us.ihmc.etherCAT.master;

import us.ihmc.etherCAT.master.EtherCATStatusCallback.TRACE_EVENT;
import us.ihmc.etherCAT.master.pipeline.LightWeightPipelineExecutor;
import us.ihmc.etherCAT.master.pipeline.LightWeightPipelineTask;
import us.ihmc.soem.generated.ec_slavet;
import us.ihmc.soem.generated.ec_state;
import us.ihmc.soem.generated.soem;

/**
 * Statemachine to do the state control for EtherCAT slaves.
 *
 * This will configure the slaves to go in OP when the master reached stable execution as well as deal with slaves that got into an error mode
 *
 * @author Jesper Smith
 */
class EtherCATStateMachine
{
   public static final long MINIMUM_JITTER_SAMPLES = 1000;

   public static boolean DEBUG = false;

   private final Master master;
   private Slave[] subdevices;

   private final LightWeightPipelineExecutor executor = new LightWeightPipelineExecutor();
   private final WaitForMasterState waitForMasterState = new WaitForMasterState();

   private long startTime = -1;

   EtherCATStateMachine(Master master)
   {
      this.master = master;
      executor.addTask(waitForMasterState);
   }

   void setSubdevices(Slave[] subdevices)
   {
      this.subdevices = subdevices;

      for (int i = 0; i < subdevices.length; i++)
      {
         SubDeviceStatePipeline subPipeline = new SubDeviceStatePipeline(master, subdevices[i]);
         subPipeline.addToExecutor(executor);
      }
   }

   void runOnce()
   {
      if (startTime < 0)
      {
         startTime = System.nanoTime();
      }

      long runTime = System.nanoTime() - startTime;
      executor.execute(runTime);
   }

   LightWeightPipelineExecutor getExecutor()
   {
      return executor;
   }

   /**
    * State to wait for the master to attain a stable, minimal jitter rate.
    */
   private class WaitForMasterState implements LightWeightPipelineTask
   {

      /**
       * Check if we have to wait
       */
      @Override
      public boolean skipTask()
      {
         long jitterEstimate = master.getJitterEstimate();

         if (!master.getDCEnabled())
         {
            return true;
         }
         else
         {
            if (master.getJitterSamples() < MINIMUM_JITTER_SAMPLES)
            {
               return false;
            }
            else if (jitterEstimate == 0 || jitterEstimate > master.getMaximumExecutionJitter())
            {
               return false;
            }
            else
            {
               return true;
            }
         }
      }

      /**
       * Nothing to do, return false to keep waiting
       */
      @Override
      public boolean execute(long runtime)
      {
         return false;
      }
   }

   void shutDown()
   {
      master.getEtherCATStatusCallback().trace(TRACE_EVENT.SWITCH_PREOP);
      ec_slavet allSlaves = soem.ecx_slave(master.getContext(), 0);
      allSlaves.setState(ec_state.EC_STATE_PRE_OP.swigValue());
      soem.ecx_writestate(master.getContext(), 0);

      master.getEtherCATStatusCallback().trace(TRACE_EVENT.CLEANUP_SLAVES);

      for (int i = 0; i < subdevices.length; i++)
      {
         subdevices[i].cleanup();
      }
   }
}
