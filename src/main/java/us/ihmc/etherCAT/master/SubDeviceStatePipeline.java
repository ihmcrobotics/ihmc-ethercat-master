package us.ihmc.etherCAT.master;

import java.util.ArrayList;

import us.ihmc.etherCAT.master.Slave.State;
import us.ihmc.etherCAT.master.pipeline.LightWeightPipelineFallTrough;
import us.ihmc.etherCAT.master.pipeline.LightWeightPipelineBin;
import us.ihmc.etherCAT.master.pipeline.LightWeightPipelineTask;
import us.ihmc.etherCAT.master.pipeline.LightWeightPipelineTransition;

public class SubDeviceStatePipeline
{
   private final Slave subDevice;
   
   
   private boolean hasReachedOp = false;
   private int previousWkc = -1;
   
   private final LightWeightPipelineBin pipelineBin;
   
   public SubDeviceStatePipeline(Master mainDevice, Slave subDevice)
   {
      this.subDevice = subDevice;
      
      
      ReadDeviceState readDeviceState = new ReadDeviceState();
      ReadRXTXErrors readRXTXErrors = new ReadRXTXErrors();
      DoSDOTransfers doSDOTransfers = new DoSDOTransfers();
      DoEtherCATStateControl doEtherCATStateControl = new DoEtherCATStateControl();
      LightWeightPipelineFallTrough finalState = new LightWeightPipelineFallTrough();
      
      LightWeightPipelineTransition stateControlTransition = new LightWeightPipelineTransition(() ->
      {
         if(subDevice.getHouseholderState() == State.OP)
         {
            hasReachedOp = true;
            return doSDOTransfers;
         }
         else if (mainDevice.isRecoveryDisabled() && hasReachedOp)
         {
            return finalState;
         }
         else
         {
            return doEtherCATStateControl;
         }
      });
      
      
            
      LightWeightPipelineTransition checkWorkingCounter = new LightWeightPipelineTransition(() ->
      {
            if(mainDevice.getActualWorkingCounter() != previousWkc)
            {
               previousWkc = mainDevice.getActualWorkingCounter();
               return readDeviceState;
            }
            else if(mainDevice.isReadRXErrorStatistics())
            {
               return readRXTXErrors;
            }
            else
            {
               return stateControlTransition;
            }
      });
      
      readDeviceState.setNextTask(readRXTXErrors);
      readRXTXErrors.setNextTask(stateControlTransition);
      doEtherCATStateControl.setNextTask(finalState);
      
      pipelineBin = new LightWeightPipelineBin(checkWorkingCounter, finalState);
      
   }
   
   public LightWeightPipelineBin getPipelineBin()
   {
      return pipelineBin;
   }

   private class ReadDeviceState extends LightWeightPipelineTask
   {

      @Override
      public boolean execute(long runtime)
      {
         subDevice.updateEtherCATState();
         return true;
      }
   }
   
   private class ReadRXTXErrors extends LightWeightPipelineTask
   {

      @Override
      public boolean execute(long runtime)
      {
         subDevice.updateRXTXStats();
         return true;
      }
      
   }
   
   private class DoEtherCATStateControl extends LightWeightPipelineTask
   {

      @Override
      public boolean execute(long runtime)
      {
         subDevice.doEtherCATStateControl(runtime);
         return true;
      }
      
   }
   
   private class DoSDOTransfers extends LightWeightPipelineTask
   {
      
      int currentSDO = 0;

      @Override
      public boolean execute(long runtime)
      {
         ArrayList<SDO> sdos = subDevice.getSDOs();
         
         if(sdos.size() > 0)
         {
            
            // Loop trough all SDO's till a transaction is made
            for(;currentSDO < sdos.size(); currentSDO++)
            {
               if(sdos.get(currentSDO).update())
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
