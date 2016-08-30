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

   protected final Slave slave;
   protected final int index;
   protected final int subindex;
   protected final int size;
   protected final ByteBuffer buffer;
  
   protected boolean send = false;
   protected boolean valid = false;
   
   
   protected SDO(Slave slave, int index, int subindex, int size)
   {
      this.slave = slave;
      this.index = index;
      this.subindex = subindex;
      this.size = size;
      buffer = ByteBuffer.allocateDirect(size);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

   }

   protected boolean canSend()
   {
      return !send;
   }
   
   /**
    * Queue a new transaction
    * 
    * @return false if the previous transaction is still ongoing
    */
   protected boolean queue()
   {
      if(send)
      {
         return false;
      }
      
      send = true;
      valid = false;
      
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
   
   /**
    * Internal function. Execute from Master EtherCAT state machine.
    * 
    * @return true if an EtherCAT transaction was made
    */
   boolean update()
   {
      if(send)
      {
         if(send() > 0)
         {
            send = false;
            valid = true;
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

}
