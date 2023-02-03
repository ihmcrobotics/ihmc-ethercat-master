package us.ihmc.etherCAT.master.pipeline;

public class LightWeightPipelineBin extends LightWeightPipelineTask
{
   
   private final LightWeightPipelineTask finalTask = new LightWeightPipelineTask()
   {
      public boolean runNextImmediatly()
      {
         return true;
      }

      @Override
      public boolean execute(long runtime)
      {
         return true;
      }
   };

   public LightWeightPipelineBin(LightWeightPipelineTask firstTask, LightWeightPipelineTask lastTask)
   {
      super.setNextTask(firstTask);
      lastTask.setNextTask(finalTask);
   }

   public final boolean runNextImmediatly()
   {
      return true;
   }

   @Override
   public final boolean execute(long runtime)
   {
      return true;
   }

   @Override
   public void setNextTask(LightWeightPipelineTask nextTask)
   {
      finalTask.setNextTask(nextTask);
   }
   
}
