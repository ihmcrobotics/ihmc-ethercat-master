package us.ihmc.etherCAT.master;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.etherCAT.master.Slave.State;
import us.ihmc.etherCAT.master.pipeline.LightWeightPipelineExecutor;
import us.ihmc.etherCAT.master.pipeline.LightWeightPipelineTask;

public class SubDeviceStatePipeline
{
   private final Master mainDevice;
   private final Slave subDevice;
   private final List<SDO> sdos;

   private final List<LightWeightPipelineTask> tasks = new ArrayList<>();

   public SubDeviceStatePipeline(Master mainDevice, Slave subDevice)
   {
      this.mainDevice = mainDevice;
      this.subDevice = subDevice;
      this.sdos = subDevice.getSDOs();

      ReadDeviceState readDeviceState = new ReadDeviceState();
      ClearRXErrors clearRXErrors = new ClearRXErrors();
      ReadRXErrors readRXErrors = new ReadRXErrors();
      DoEtherCATStateControl doEtherCATStateControl = new DoEtherCATStateControl();
      DoSDOTransfers doSDOTransfers = new DoSDOTransfers();

      tasks.add(readDeviceState);
      tasks.add(clearRXErrors);
      tasks.add(readRXErrors);
      tasks.add(doEtherCATStateControl);
      tasks.add(doSDOTransfers);
   }

   public void addToExecutor(LightWeightPipelineExecutor executor)
   {
      executor.addTasks(tasks);
   }

   /**
    * Check if the master working counter is the expected working counter and the device is in OP state
    * @return true if wkc is expected and slave is in OP
    */
   private boolean inNormalOperation()
   {
      return subDevice.getState() == State.OP && mainDevice.getActualWorkingCounter() == mainDevice.getExpectedWorkingCounter();
   }

   private class ReadDeviceState implements LightWeightPipelineTask
   {
      @Override
      public boolean execute(long runtime)
      {
         subDevice.updateEtherCATState();
         return true;
      }

      /**
       * Skip this task when the working counter matches the previous working counter and the device is in OP mode
       */
      @Override
      public boolean skipTask()
      {
         return inNormalOperation();

      }
   }

   private class ClearRXErrors implements LightWeightPipelineTask
   {
      public boolean cleared = false;

      @Override
      public boolean execute(long runtime)
      {
         if (subDevice.clearRXErrors())
         {
            cleared = true;
            return true;
         }
         else
         {
            return false;
         }
      }

      /**
       * Skip if the RX errors are already cleared
       */
      @Override
      public boolean skipTask()
      {
         return cleared;
      }
   }

   private class ReadRXErrors implements LightWeightPipelineTask
   {

      @Override
      public boolean execute(long runtime)
      {
         subDevice.updateRXTXStats();
         return true;
      }

      /**
       * Skip if reading rx error statistics is disabled
       */
      @Override
      public boolean skipTask()
      {
         if (!mainDevice.isReadRXErrorStatistics())
         {
            // Reading RX Errors statistics is disabled
            return true;
         }
         else if (subDevice.getHouseholderState() == State.OFFLINE)
         {
            // Don't try to read these statistics when the subdevice is offline
            return true;
         }
         else
         {
            return false;
         }
      }

   }

   private class DoEtherCATStateControl implements LightWeightPipelineTask
   {
      private boolean hasReachedOp = false;

      @Override
      public boolean execute(long runtime)
      {
         subDevice.doEtherCATStateControl(runtime);
         return true;
      }

      /**
       * Skip this task if the slave is in OP, or has reached OP and recover has been disabled
       */
      @Override
      public boolean skipTask()
      {
         if (subDevice.getHouseholderState() == State.OP)
         {
            // No need to state control if the state is OP
            hasReachedOp = true;
            return true;
         }
         else if (mainDevice.isRecoveryDisabled() && hasReachedOp)
         {
            // Recovery is disabled and the state has been in OP. Do not retry to enable the slave
            return true;
         }
         else
         {
            return false;
         }

      }

   }

   private class DoSDOTransfers implements LightWeightPipelineTask
   {

      /**
       * Skip SDO transfer if the wkc is wrong
       */
      public boolean skipTask()
      {
         if (!inNormalOperation())
         {
            // Slave is not in OP, or there is something wrong with the ethercat chain. Skip SDO transmission
            return true;
         }
         else if (sdos.isEmpty())
         {
            // No SDOs defined
            return true;
         }
         else
         {
            for (int i = 0; i < sdos.size(); i++)
            {
               if (sdos.get(i).isTransferPending())
               {
                  // An SDO needs data transfer
                  return false;
               }
            }

            return true;
         }
      }

      int currentSDO = 0;

      @Override
      public boolean execute(long runtime)
      {
         ArrayList<SDO> sdos = subDevice.getSDOs();

         if (sdos.size() > 0)
         {

            // Loop trough all SDO's till a transaction is made

            while (currentSDO < sdos.size())
            {
               int sdoToUpdate = currentSDO;
               currentSDO++;

               if (sdos.get(sdoToUpdate).updateFromStatemachineThread())
               {
                  break;
               }
            }

            if (currentSDO >= sdos.size())
            {
               currentSDO = 0;
               return true;
            }
            else
            {
               return false;
            }
         }
         else
         {
            return true;
         }
      }
   }

}
