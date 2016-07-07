package us.ihmc.soem.wrapper;

import java.nio.ByteBuffer;

public class ReadSDO extends SDO
{
   
   public ReadSDO(Slave slave, int index, int subindex, int size)
   {
      super(slave, index, subindex, size);
   }
   
   /**
    * Request new data if hasNewData() is false and no data has previously been requested.
    */
   public void update()
   {
      requestUpdateOnNextTick();
   }
   

   
   /**
    * Get current data if hasNewData() is true; Else throw runtimeException. 
    * 
    * After this call, hasNewData() will be false
    * 
    * @return latest data.
    * @throws RuntimeException if hasNewData is false
    */
   private ByteBuffer getData()
   {
      if(checkIfRequestSuccess())
      {
         return buffer;
      }
      
      throw new RuntimeException("No new SDO data available. Please only call when hasNewData() is true");
   }

   public boolean hasNewData()
   {
      return transactionIsDone();
   }
   
   
   public long getLong()
   {
      return getData().getLong();
   }
   
   public int getInt()
   {
      return getData().getInt();
   }
   
   public long getUnsignedInt()
   {
      return getInt() & 0xFFFFFFFFL;
   }
   
   public short getShort()
   {
      return getData().getShort();
   }
   
   public int getUnsignedShort()
   {
      return getShort() & 0xFFFF;
   }
   
   public byte getByte()
   {
      return getData().get();
   }
   
   public int getUnsignedByte()
   {
      return getByte() & 0xFF;
   }

   @Override
   protected boolean doTransaction()
   {
      slave.readSDOToBuffer(index, subindex, size, buffer);
      return true;
   }
}
