package us.ihmc.soem.wrapper;

import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Base objects for using SDO objects in cyclic operations.
 * 
 * The SDO will be delegated to the house holding objects and can be polled for data.
 * 
 * @author Jesper Smith
 *
 */
public abstract class SDO
{

   protected enum State
   {
      REQUEST_NEW_DATA, REQUEST_SUCCESS, IDLE
   }

   protected final Slave slave;
   protected final int index;
   protected final int subindex;
   protected final int size;
   protected final ByteBuffer buffer;
   private final AtomicReference<State> state = new AtomicReference<>(State.IDLE);

   protected SDO(Slave slave, int index, int subindex, int size)
   {
      this.slave = slave;
      this.index = index;
      this.subindex = subindex;
      this.size = size;
      buffer = ByteBuffer.allocateDirect(size);

   }

   protected boolean checkIfRequestSuccess()
   {
      return state.compareAndSet(State.REQUEST_SUCCESS, State.IDLE);
   }

   protected void requestUpdateOnNextTick()
   {
      state.compareAndSet(State.IDLE, State.REQUEST_NEW_DATA);
   }

   protected boolean transactionIsDone()
   {
      return state.get() == State.REQUEST_SUCCESS;
   }

   protected boolean sdoIsIdle()
   {
      return state.get() == State.IDLE;
   }

   protected abstract boolean doTransaction();

   void updateInMasterThread()
   {
      State currentState = state.get();

      switch (currentState)
      {
      case REQUEST_NEW_DATA:
         if (doTransaction())
         {
            state.set(State.REQUEST_SUCCESS);
         }
         break;

      case REQUEST_SUCCESS:
      case IDLE:
         break;
      }
   }

}
