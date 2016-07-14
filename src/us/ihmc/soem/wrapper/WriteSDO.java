package us.ihmc.soem.wrapper;

/**
 * SDO write delegator class
 * 
 * The SDO will be delegated to the house holding objects and can be polled for data.
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
   public boolean isReady()
   {
      return sdoIsIdle();
   }

   /**
    * Checks if data was written and go back to idle state
    * 
    * @return true if the data was written
    */
   public boolean checkIfSuccessful()
   {
      return transactionIsDone();
   }

   /**
    * Write value to SDO. 
    * 
    * Data can only be written if isReady() is true. Use checkIfSuccessfull to see if a previous write is done and reset.
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
    * Data can only be written if isReady() is true. Use checkIfSuccessfull to see if a previous write is done and reset.
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
    * Data can only be written if isReady() is true. Use checkIfSuccessfull to see if a previous write is done and reset.
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
    * Data can only be written if isReady() is true. Use checkIfSuccessfull to see if a previous write is done and reset.
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
    * Data can only be written if isReady() is true. Use checkIfSuccessfull to see if a previous write is done and reset.
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
    * Data can only be written if isReady() is true. Use checkIfSuccessfull to see if a previous write is done and reset.
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
      slave.writeSDO(index, subindex, buffer);
      return true;
   }
}
