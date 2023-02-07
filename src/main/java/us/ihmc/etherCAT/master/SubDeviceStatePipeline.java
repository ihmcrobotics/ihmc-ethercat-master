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
   
   private boolean hasReachedOp = false;
   private int previousWkc = -1;
   
   
   private final List<LightWeightPipelineTask> tasks = new ArrayList<>();
   
   
   public SubDeviceStatePipeline(Master mainDevice, Slave subDevice)
   {
      this.mainDevice = mainDevice;
      this.subDevice = subDevice;
      this.sdos = subDevice.getSDOs();
      
      
      ReadDeviceState readDeviceState = new ReadDeviceState();
      ReadRXTXErrors readRXTXErrors = new ReadRXTXErrors();
      DoEtherCATStateControl doEtherCATStateControl = new DoEtherCATStateControl();
      DoSDOTransfers doSDOTransfers = new DoSDOTransfers();
      
      
      
      tasks.add(readDeviceState);
      tasks.add(readRXTXErrors);
      tasks.add(doEtherCATStateControl);
      tasks.add(doSDOTransfers);
   }
   
   public void addToExecutor(LightWeightPipelineExecutor executor)
   {
      executor.addTasks(tasks);
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
       * Skip this task when the working counter matches the previous working counter
       */
      @Override
      public boolean skipTask()
      {
         if(mainDevice.getActualWorkingCounter() == previousWkc)
         {
            return false;
         }
         else
         {
            previousWkc = mainDevice.getActualWorkingCounter();
            return false;
         }

      }
   }
   
   private class ReadRXTXErrors implements LightWeightPipelineTask
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
         return !mainDevice.isReadRXErrorStatistics();
      }
      
   }
   
   private class DoEtherCATStateControl implements LightWeightPipelineTask
   {

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
         if(subDevice.getHouseholderState() == State.OP)
         {
            hasReachedOp = true;
            return true;
         }
         else if (mainDevice.isRecoveryDisabled() && hasReachedOp)
         {
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
         if( mainDevice.getActualWorkingCounter() != mainDevice.getExpectedWorkingCounter() )
         {           
            return true;
         }
         else if (subDevice.getHouseholderState() != State.OP)
         {
            return true;
         }
         else if (sdos.isEmpty())
         {
            return true;
         }
         else
         {
            for(int i = 0; i < sdos.size(); i++)
            {
               if(sdos.get(i).isTransferPending())
               {
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
         
         if(sdos.size() > 0)
         {
            
            // Loop trough all SDO's till a transaction is made
            
            while(currentSDO < sdos.size())
            {
               int sdoToUpdate = currentSDO;
               currentSDO++;
               
               if(sdos.get(sdoToUpdate).updateFromStatemachineThread())
               {
                  break;
               }
            }
            
            if(currentSDO >= sdos.size())
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
