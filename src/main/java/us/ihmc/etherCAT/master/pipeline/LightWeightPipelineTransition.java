package us.ihmc.etherCAT.master.pipeline;

import java.util.function.Supplier;

public class LightWeightPipelineTransition extends LightWeightPipelineTask
{
   private final Supplier<LightWeightPipelineTask> nextSupplier;
   
   public LightWeightPipelineTransition(Supplier<LightWeightPipelineTask> nextSupplier)
   {
      this.nextSupplier = nextSupplier;
   }


   @Override
   public final boolean runNextImmediatly()
   {
      return true;
   }
   
   
   final LightWeightPipelineTask next()
   {
      return nextSupplier.get();
   }
 
   
   public final boolean execute(long runtime)
   {
      return true;
   }
   
   
}
