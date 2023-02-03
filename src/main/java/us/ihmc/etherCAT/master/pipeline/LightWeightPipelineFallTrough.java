package us.ihmc.etherCAT.master.pipeline;

public class LightWeightPipelineFallTrough extends LightWeightPipelineTask
{

   @Override
   public boolean execute(long runtime)
   {
      return true;
   }
   
   public boolean runNextImmediatly()
   {
      return true;
   }
   
}