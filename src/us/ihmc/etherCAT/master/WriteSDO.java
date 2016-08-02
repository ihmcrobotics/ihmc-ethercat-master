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
    * Checks if new data can be written 
    * 
    * @return true if ready for a write request
    */
   private boolean isReady()
   {
      return sdoIsIdle();
   }

   /**
    * Checks if data was written and go back to idle state.
    * 
    * Only the first call after a successful write will return true, subsequent calls will return false
    * 
    * @return true if the data was written
    */
   public boolean dataHasBeenWritten()
   {
      return checkIfRequestSuccess();
   }
   
   /**
    * Check if data can be written.
    * 
    * - If no data has ever been written, return true
    * - If data has successfully been written, return true
    * - If data is currently been written from a previous command, return false
    * 
    * As side effect, calling this function will result in dataHasBeenWritten to be false. 
    * If the result of a previous write needs to be checked, use dataHsBeenWritten()
    * 
    * @return true if new data can be written
    */
   public boolean canWriteData()
   {
      return dataHasBeenWritten() || isReady();
   }

   /**
    * Write value to SDO. 
    * 
    * Data can only be written if canWriteData is true.
    * 
    * @param value
    */
   public void write(byte value)
   {
      if (!isReady())
      {
         throw new RuntimeException("Cannot write new value to SDO, SDO is not ready");
      }

      buffer.clear();
      buffer.put(value);

      requestUpdateOnNextTick();
   }

   /**
    * Write value to SDO. 
    * 
    * Data can only be written if canWriteData is true.
    * 
    * @param value
    */
   public void write(short value)
   {
      if (!isReady())
      {
         throw new RuntimeException("Cannot write new value to SDO, SDO is not ready");
      }

      buffer.clear();
      buffer.putShort(value);

      requestUpdateOnNextTick();
   }

   /**
    * Write value to SDO. 
    * 
    * Data can only be written if canWriteData is true.
    * 
    * @param value
    */
   public void write(int value)
   {
      if (!isReady())
      {
         throw new RuntimeException("Cannot write new value to SDO, SDO is not ready");
      }

      buffer.clear();
      buffer.putInt(value);

      requestUpdateOnNextTick();
   }

   /**
    * Write value to SDO. 
    * 
    * Data can only be written if canWriteData is true.
    * 
    * @param value
    */
   public void write(long value)
   {
      if (!isReady())
      {
         throw new RuntimeException("Cannot write new value to SDO, SDO is not ready");
      }

      buffer.clear();
      buffer.putLong(value);

      requestUpdateOnNextTick();
   }

   /**
    * Write value to SDO. 
    * 
    * Data can only be written if canWriteData is true.
    * 
    * @param value
    */
   public void write(double value)
   {
      if (!isReady())
      {
         throw new RuntimeException("Cannot write new value to SDO, SDO is not ready");
      }

      buffer.clear();
      buffer.putDouble(value);

      requestUpdateOnNextTick();
   }

   /**
    * Write value to SDO. 
    * 
    * Data can only be written if canWriteData is true.
    * 
    * @param value
    */
   public void write(float value)
   {
      if (!isReady())
      {
         throw new RuntimeException("Cannot write new value to SDO, SDO is not ready");
      }

      buffer.clear();
      buffer.putFloat(value);

      requestUpdateOnNextTick();
   }

   @Override
   protected boolean doTransaction()
   {
      return slave.writeSDO(index, subindex, buffer) != 0;
   }

}
