package us.ihmc.etherCAT.master.pipeline;

import java.util.ArrayList;
import java.util.List;

public class LightWeightPipelineExecutor
{
   private final List<LightWeightPipelineTask> tasks = new ArrayList<>();
   private int currentTaskIndex = 0;
   
   private int lastExecutedTaskIndex = 0;

   public LightWeightPipelineExecutor()
   {
   }

   private LightWeightPipelineTask currentTask()
   {

      return tasks.get(currentTaskIndex);
   }
   
   public void addTask(LightWeightPipelineTask task)
   {
      tasks.add(task);
   }
   
   public void addTasks(List<LightWeightPipelineTask> tasks)
   {
      this.tasks.addAll(tasks);
   }

   public void execute(long runtime)
   {
      while (currentTask().skipTask())
      {
         moveToNextTask();
      }

      lastExecutedTaskIndex = currentTaskIndex;
      if (currentTask().execute(runtime))
      {
         moveToNextTask();
      }
      System.out.println(getLastExectutedTaskName());
   }
   
   public int getLastExecutedTaskIndex()
   {
      return lastExecutedTaskIndex;
   }
   
   public String getLastExectutedTaskName()
   {
      return tasks.get(lastExecutedTaskIndex).getClass().getSimpleName();
   }

   private void moveToNextTask()
   {
      currentTaskIndex++;
      if (currentTaskIndex >= tasks.size())
      {
         currentTaskIndex = 0;
      }
   }
}
