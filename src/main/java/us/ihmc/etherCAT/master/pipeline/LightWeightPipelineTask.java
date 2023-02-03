package us.ihmc.etherCAT.master.pipeline;

public abstract class LightWeightPipelineTask
{
   private LightWeightPipelineTask nextTask;
   
   /**
    * If true, do not run execute but run the next task immediatly
    * 
    * @return
    */
   public boolean runNextImmediatly()
   {
      return false;
   }
   
   
   /**
    * Execute this task
    * @param runtime TODO
    * @return True if the task is finished
    */
   public abstract boolean execute(long runtime);
   
   
   public void setNextTask(LightWeightPipelineTask nextTask)
   {
      this.nextTask = nextTask;
   }
   
   
   /**
    * Return the next task
    * @return
    */
   LightWeightPipelineTask next()
   {
      return nextTask;
   }
}
