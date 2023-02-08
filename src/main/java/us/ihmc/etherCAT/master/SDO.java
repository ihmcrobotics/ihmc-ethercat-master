package us.ihmc.etherCAT.master;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

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
   private enum CyclicState
   {
      WAITING_FOR_USER_DATA, QUEUED, WAITING_FOR_TRANSFER
   }

   private enum StatemachineState
   {
      IDLE, DO_TRANSFER, TRANSFER_DONE, TRANSFER_FAILED
   }

   protected final Slave slave;
   protected final int index;
   protected final int subindex;
   protected final int size;

   protected final ByteBuffer buffer;

   protected CyclicState cyclicState = CyclicState.WAITING_FOR_USER_DATA;
   protected StatemachineState statemachineState = StatemachineState.IDLE;

   private boolean valid = false;

   protected SDO(Slave slave, int index, int subindex, int size)
   {
      this.slave = slave;
      this.index = index;
      this.subindex = subindex;
      this.size = size;

      buffer = ByteBuffer.allocateDirect(size);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

   }

   /**
    * Data can be send if there is no transfer in progress
    * @return
    */
   protected boolean canSend()
   {
      return cyclicState == CyclicState.WAITING_FOR_USER_DATA;
   }

   /**
    * Queue a new transaction
    * 
    * @return false if the previous transaction is still ongoing
    */
   protected boolean queue()
   {
      if (canSend())
      {
         cyclicState = CyclicState.QUEUED;
         valid = false;
      }
      else
      {
         return false;
      }

      return true;

   }

   /**
    * The last queued SDO transaction was succesful and no new transaction has been queued
    * 
    * @return true if the data is valid
    */
   public boolean isValid()
   {
      return valid;
   }

   /**
    * Send SDO request
    * 
    * @return working counter
    */
   protected abstract int send();

   boolean isTransferPending()
   {
      return statemachineState == StatemachineState.DO_TRANSFER;
   }

   /**
    * Internal function. Execute from Master EtherCAT state machine.
    * 
    * @return true if an EtherCAT transaction was made
    */
   boolean updateFromStatemachineThread()
   {
      if (statemachineState == StatemachineState.DO_TRANSFER)
      {
         if (send() > 0)
         {
            statemachineState = StatemachineState.TRANSFER_DONE;
         }
         else
         {
            statemachineState = StatemachineState.TRANSFER_FAILED;
         }
         return true;
      }
      else
      {
         return false;
      }
   }

   Slave getSlave()
   {
      return slave;
   }

   @Override
   public String toString()
   {
      return "SDO[Slave: " + slave + "]; Address: " + EtherCATStatusCallback.hex(index) + ":" + EtherCATStatusCallback.hex(subindex) + "]";
   }

   /**
    * This function is called when both threads are guaranteed to not read/write anything. Therefore, we can 
    */
   public void syncDataWithStatemachineThread()
   {
      switch (cyclicState)
      {
         case WAITING_FOR_USER_DATA:
            break;
         case QUEUED:
            statemachineState = StatemachineState.DO_TRANSFER;
            cyclicState = CyclicState.WAITING_FOR_TRANSFER;
            break;
         case WAITING_FOR_TRANSFER:
            checkStatemachineState();
            break;
      }

   }

   private void checkStatemachineState()
   {
      switch (statemachineState)
      {
         case IDLE:
         case DO_TRANSFER:
            break;
         case TRANSFER_DONE:
            valid = true;
            cyclicState = CyclicState.WAITING_FOR_USER_DATA;
            break;
         case TRANSFER_FAILED:
            valid = false;
            cyclicState = CyclicState.WAITING_FOR_USER_DATA;
            break;
      }
   }

}
