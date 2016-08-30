package us.ihmc.etherCAT.master;

import java.nio.ByteBuffer;


/**
 * SDO read delegator class for SDO transfer
 * 
 * The SDO will be delegated to the house holding thread and can be polled for data.
 * This class is meant to be used to read SDO's during execution of the realtime EtherCAT
 * transfer. To read SDO's for slave configuration purposes, call slave.readSDO() directly from the 
 * configure() function overridden from Slave. 
 * 
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
    * Request new data
    * 
    * @return false if another request is still running
    */
   public boolean requestNewData()
   {
      return queue();
   }

   
   /**
    * Get current data if valid() is true; Else throw runtimeException. 
    * 
    * @return latest data.
    * @throws RuntimeException if valid is false
    */
   private ByteBuffer getData()
   {
      if(isValid())
      {
         return buffer;
      }
      
      throw new RuntimeException("No new SDO data available. Please only call when valid() is true");
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
   protected int send()
   {
      return slave.readSDOToBuffer(index, subindex, size, buffer);
   }
}
