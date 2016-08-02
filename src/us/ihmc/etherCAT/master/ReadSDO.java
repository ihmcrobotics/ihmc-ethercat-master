package us.ihmc.etherCAT.master;

import java.nio.ByteBuffer;


/**
 * SDO read delegator class for asynchronous SDO transfer
 * 
 * The SDO will be delegated to the house holding thread and can be polled for data.
 * This class is meant to be used to read SDO's during execution of the realtime EtherCAT
 * transfer. To read SDO's for slave configuration purposes, call slave.readSDO() directly from the 
 * configure() function overridden from Slave. 
 * 
 * The public class interface is not thread safe. Calling requestNewData(), hasNewData()
 * and the various getters from multiple threads on the same object will result in data
 * corruption. However, SDO's can be read from outside the EtherCAT thread.
 * 
 * @author Jesper Smith
 *
 */
public class ReadSDO extends SDO
{

   /**
    * Create new Read SDO object
    * 
    * @param slave Slave to read
    * @param index Index of SDO
    * @param subindex SubIndex of SDO
    * @param size Size of SDO data
    */
   public ReadSDO(Slave slave, int index, int subindex, int size)
   {
      super(slave, index, subindex, size);
   }
   
   /**
    * Request new data if hasNewData() is false and no data has previously been requested.
    */
   public void requestNewData()
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

   /**
    * Check if new data is available for this SDO.
    * 
    * @return true if new data is available.
    */
   public boolean hasNewData()
   {
      return transactionIsDone();
   }
   
   /**
    * 
    * @return Data as long
    */
   public long getLong()
   {
      return getData().getLong();
   }
   

   /**
    * 
    * @return Data as int
    */
   public int getInt()
   {
      return getData().getInt();
   }
   

   /**
    * 
    * @return Data as unsigned int
    */
   public long getUnsignedInt()
   {
      return getInt() & 0xFFFFFFFFL;
   }
   

   /**
    * 
    * @return Data as short
    */
   public short getShort()
   {
      return getData().getShort();
   }
   

   /**
    * 
    * @return Data as unsigned short
    */
   public int getUnsignedShort()
   {
      return getShort() & 0xFFFF;
   }
   

   /**
    * 
    * @return Data as byte
    */
   public byte getByte()
   {
      return getData().get();
   }
   

   /**
    * 
    * @return Data as usigned byte
    */
   public int getUnsignedByte()
   {
      return getByte() & 0xFF;
   }
   
   /**
    * 
    * @return Data as double
    */
   public double getDouble()
   {
      return getData().getDouble();
   }

   /**
    * 
    * @return Data as float
    */
   public double getFloat()
   {
      return getData().getFloat();
   }

   @Override
   protected boolean doTransaction()
   {
      return slave.readSDOToBuffer(index, subindex, size, buffer) != 0;
   }
}
