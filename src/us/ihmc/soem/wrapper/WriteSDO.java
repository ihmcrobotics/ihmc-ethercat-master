package us.ihmc.soem.wrapper;

public class WriteSDO extends SDO
{

   public WriteSDO(Slave slave, int index, int subindex, int size)
   {
      super(slave, index, subindex, size);
   }

   public boolean isReady()
   {
      return sdoIsIdle();
   }

   public boolean checkIfSuccessful()
   {
      return transactionIsDone();
   }

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
