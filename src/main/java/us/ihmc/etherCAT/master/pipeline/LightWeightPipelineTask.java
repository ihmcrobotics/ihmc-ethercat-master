package us.ihmc.etherCAT.master.pipeline;

public interface LightWeightPipelineTask
{   
   /**
    * If true, do not run execute but run the next task immediately
    * 
    * @return
    */
   public boolean skipTask();
   
   
   /**
    * Execute this task
    * @param runtime TODO
    * @return True if the task is finished
    */
   public boolean execute(long runtime);
}
