package us.ihmc.etherCAT.master;

/**
 * SDO write delegator class for asynchronous SDO transfer
 * 
 * The SDO will be delegated to the house holding thread and can be polled for data.
 * This class is meant to be used to write SDO's during execution of the realtime EtherCAT
 * transfer. To write SDO's for slave configuration purposes, call slave.writeSDO() directly from the 
 * configure() function overridden from Slave. 
 * 
 * The public class interface is not thread safe. Calling dataHasBeenWritten, canWriteData and write() 
 * from multiple threads on the same object will result in  data corruption. However, SDO's can be written
 * from outside the EtherCAT thread.
 * 
 * @author Jesper Smith
 *
 */
public class WriteSDO extends SDO
{
   /**
    * New SDO for writing data.
    * 
    * @param slave Slave
    * @param index Index of the SDO
    * @param subindex Subindex of the SDO
    * @param size Size of the data to be written
    */
   public WriteSDO(Slave slave, int index, int subindex, int size)
   {
      super(slave, index, subindex, size);
   }

   /**
    * Write value to SDO. 
    * 
    * 
    * @param value
    */
   public boolean write(byte value)
   {
      if (!canSend())
      {
         return false;
      }

      buffer.clear();
      buffer.put(value);

      return queue();
   }

   /**
    * Write value to SDO. 
    * 
    * Data can only be written if canWriteData is true.
    * 
    * @param value
    */
   public boolean write(short value)
   {
      if (!canSend())
      {
         return false;
      }

      buffer.clear();
      buffer.putShort(value);

      return queue();
   }

   /**
    * Write value to SDO. 
    * 
    * Data can only be written if canWriteData is true.
    * 
    * @param value
    */
   public boolean write(int value)
   {
      if (!canSend())
      {
         return false;
      }

      buffer.clear();
      buffer.putInt(value);

      return queue();
   }

   /**
    * Write value to SDO. 
    * 
    * Data can only be written if canWriteData is true.
    * 
    * @param value
    */
   public boolean write(long value)
   {
      if (!canSend())
      {
         return false;
      }

      buffer.clear();
      buffer.putLong(value);

      return queue();
   }

   /**
    * Write value to SDO. 
    * 
    * Data can only be written if canWriteData is true.
    * 
    * @param value
    */
   public boolean write(double value)
   {
      if (!canSend())
      {
         return false;
      }

      buffer.clear();
      buffer.putDouble(value);

      return queue();
   }

   /**
    * Write value to SDO. 
    * 
    * Data can only be written if canWriteData is true.
    * 
    * @param value
    */
   public boolean write(float value)
   {
      if (!canSend())
      {
         return false;
      }

      buffer.clear();
      buffer.putFloat(value);

      return queue();
   }

   @Override
   protected int send()
   {
      return slave.writeSDO(index, subindex, buffer);
   }

}
