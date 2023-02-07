package us.ihmc.etherCAT.master;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.LockSupport;

import us.ihmc.affinity.Processor;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;

/**
 * Class to run the EtherCAT state control in a separate thread concurrent to doControl in the EtherCATRealtimeThread
 * 
 * A lightweight locking mechanism is implemented to simplify execution. Priority inversion is possible, therefore care should be taken that this
 * thread is ran with equal or higher priority than all other threads.
 * 
 * @author Jesper Smith
 *
 */
class EtherCATStatemachineThread
{
   private static final long PARK_TIME_NANOS = 10000000L; //10 milliseconds

   private final static int STARTING = 0;
   private final static int IDLE = 1;
   private final static int CYCLIC_RUNNING = 2;
   private final static int CYCLIC_DONE = 3;
   private final static int STATE_CONTROL_RUNNING = 4;
   private final static int STATE_CONTROL_DONE = 5;
   private final static int SHUTDOWN = 6;

   private final AtomicInteger state = new AtomicInteger();
   private final RealtimeThread thread;

   private Thread javaThread = null;

   private final Master master;

   private long durationInThread = 0;
   private long durationInCyclic = 0;

   public EtherCATStatemachineThread(PriorityParameters priorityParameters, Master master)
   {
      this.thread = new RealtimeThread(priorityParameters, this::runStatemachineThread, getClass().getSimpleName());

      this.master = master;
      state.set(STARTING);
   }
   
   public void start()
   {
      this.thread.start();      
   }

   public boolean tryLockCyclic()
   {
      int currentState = state.get();

      if (currentState == SHUTDOWN)
      {
         throw new RuntimeException("Thread has shut down");
      }
      else if (currentState == STARTING)
      {
         return false;
      }
      else if (currentState == STATE_CONTROL_RUNNING)
      {
         return false;
      }
      else
      {
         return state.compareAndSet(currentState, CYCLIC_RUNNING);
      }
   }

   public void releaseCyclicAndStartStateControl()
   {
      // Set the state to CYCLIC_DONE. If the state is not CYCLIC_RUNNING, crash
      if (state.compareAndSet(CYCLIC_RUNNING, CYCLIC_DONE))
      {
         // Copy the duration variable here, as nothing will access it.
         durationInCyclic = durationInThread;

         LockSupport.unpark(javaThread);
      }
      else
      {
         throw new RuntimeException("Illegal State " + state.get());
      }

   }

   private void executeStatemachine()
   {
      long startTime = System.nanoTime();
      master.doEtherCATStateControl();
      durationInThread = System.nanoTime() - startTime;
   }

   private void runStatemachineThread()
   {
      // Store the java thread and switch to idle state
      javaThread = Thread.currentThread();
      if (!state.compareAndSet(STARTING, IDLE))
      {
         throw new RuntimeException("Illegal state " + state.get());
      }

      while (true)
      {
         // Park for a short amount of time, to avoid deadlocks
         LockSupport.parkNanos(PARK_TIME_NANOS);

         // Check if we want to shutdown first
         int currentState = state.get();
         if (currentState == SHUTDOWN)
         {
            break;
         }
         
         // If the state is CYCLIC_DONE, start the statemachine
         if (state.compareAndSet(CYCLIC_DONE, STATE_CONTROL_RUNNING))
         {
            try
            {
               executeStatemachine();
            }
            finally
            {
               if (!state.compareAndSet(STATE_CONTROL_RUNNING, STATE_CONTROL_DONE))
               {
                  if(currentState == SHUTDOWN)
                  {
                     break;
                  }
                  
                  throw new RuntimeException("Illegal state " + state.get());
               }
            }
         }
      }

      master.shutdown();
   }

   public void setAffinity(Processor... processors)
   {
      thread.setAffinity(processors);
   }

   public long getDuration()
   {
      return durationInCyclic;
   }

   public void shutdown()
   {
      state.set(SHUTDOWN);
   }

}
