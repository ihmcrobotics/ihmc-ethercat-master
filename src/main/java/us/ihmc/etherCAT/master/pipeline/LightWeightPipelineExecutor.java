package us.ihmc.etherCAT.master.pipeline;

public class LightWeightPipelineExecutor
{
   private final LightWeightPipelineTask initialTask;

   private LightWeightPipelineTask currentTask;

   public LightWeightPipelineExecutor(LightWeightPipelineTask initialTask)
   {
      this.initialTask = initialTask;
      this.currentTask = initialTask;
   }

   public void execute(long runtime)
   {
      System.out.println("--");
      System.out.println(currentTask);


      while (currentTask.runNextImmediatly())
      {
         debug("Falling trough ", currentTask);
         moveToNextTask(currentTask.next());
      }

      debug("Running ", currentTask);
      if (currentTask.execute(runtime))
      {
         moveToNextTask(currentTask.next());
      }

   }

   private void moveToNextTask(LightWeightPipelineTask next)
   {
      currentTask = next;
      
      if (currentTask == null)
      {
         System.out.println("Wrapping around, running initial task");
         currentTask = initialTask;
      }
   }

   private void debug(String msg, LightWeightPipelineTask task)
   {
      System.out.println(msg + " " + task.getClass().getSimpleName());
   }

}
