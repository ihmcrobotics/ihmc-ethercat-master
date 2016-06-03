package us.ihmc.soem.wrapper;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import us.ihmc.soem.generated.ec_slavet;
import us.ihmc.soem.generated.ecx_contextt;
import us.ihmc.soem.generated.soem;
import us.ihmc.soem.generated.soemConstants;
import us.ihmc.soem.wrapper.SyncManager.MailbusDirection;

public class Slave
{
   private final int aliasAddress;
   private final int configAddress;
   private final ByteBuffer sdoBuffer = ByteBuffer.allocateDirect(12);

   private final SyncManager[] syncManagers = new SyncManager[4];
   
   private ecx_contextt context;
   private ec_slavet ec_slave;
   private int slaveIndex;
   

   public Slave(int aliasAddress, int configAddress)
   {
      this.aliasAddress = aliasAddress;
      this.configAddress = configAddress;

      sdoBuffer.order(ByteOrder.nativeOrder()); // EtherCAT is a big-endian protocol, but this has to be LITTLE_ENDIAN
   }

   void configure(ecx_contextt context, ec_slavet slave, int slaveIndex)
   {
      this.context = context;
      this.ec_slave = slave;
      this.slaveIndex = slaveIndex;
   

      
      for(int i = 0; i < syncManagers.length; i++)
      {
         syncManagers[i].configure(this);
      }

      if(ec_slave.getIstartbit() != 0 || ec_slave.getOstartbit() != 0)
      {
         throw new RuntimeException("Cannot configure slaves with non-zero start bits. Current slave is " + slave.getName());
      }
      
   }
   
   public long getInputBytes() 
   {
      return ec_slave.getIbytes();
   }
   
   public long getOutputBytes()
   {
      return ec_slave.getObytes();
   }
   
   
   
   public void registerSyncManager(SyncManager syncManager)
   {
      if(syncManagers[syncManager.getIndex()] != null)
      {
         throw new RuntimeException("SyncManager " + syncManager.getIndex() + " has already been registered");
      }
      
      syncManagers[syncManager.getIndex()] = syncManager;
   }

   public int writeSDO(int index, int subIndex, double value)
   {
      sdoBuffer.clear();
      sdoBuffer.putDouble(0, value);
      return soem.ecx_SDOwrite(context, slaveIndex, index, (short) subIndex, (short) 0, 8, sdoBuffer, soemConstants.EC_TIMEOUTRXM);
   }

   public int writeSDO(int index, int subIndex, long value)
   {
      sdoBuffer.clear();
      sdoBuffer.putLong(0, value);
      return soem.ecx_SDOwrite(context, slaveIndex, index, (short) subIndex, (short) 0, 8, sdoBuffer, soemConstants.EC_TIMEOUTRXM);
   }
   
   public int writeSDO(int index, int subIndex, float value)
   {
      sdoBuffer.clear();
      sdoBuffer.putFloat(0, value);
      return soem.ecx_SDOwrite(context, slaveIndex, index, (short) subIndex, (short) 0, 4, sdoBuffer, soemConstants.EC_TIMEOUTRXM);
   }

   public int writeSDO(int index, int subIndex, int value)
   {
      sdoBuffer.clear();
      sdoBuffer.putInt(0, value);
      return soem.ecx_SDOwrite(context, slaveIndex, index, (short) subIndex, (short) 0, 4, sdoBuffer, soemConstants.EC_TIMEOUTRXM);
   }

   public int writeSDO(int index, int subIndex, short value)
   {
      sdoBuffer.clear();
      sdoBuffer.putShort(0, value);
      return soem.ecx_SDOwrite(context, slaveIndex, index, (short) subIndex, (short) 0, 2, sdoBuffer, soemConstants.EC_TIMEOUTRXM);
   }

   public int writeSDO(int index, int subIndex, byte value)
   {
      sdoBuffer.clear();
      sdoBuffer.put(0, value);
      return soem.ecx_SDOwrite(context, slaveIndex, index, (short) subIndex, (short) 0, 1, sdoBuffer, soemConstants.EC_TIMEOUTRXM);
   }
   
   /**
    * Read a SDO value to a buffer. 
    * 
    * The first four bytes in the buffer are the SDO length as integer. Position and limit are set accordingly.
    * 
    * @param index
    * @param subIndex
    * @param sdoBuffer
    * 
    * @return working counter
    */
   public int readSDOToBuffer(int index, int subIndex, int size, ByteBuffer sdoBuffer)
   {
      int wc = soem.ecx_SDOread_java_helper(context, slaveIndex, index, (short) subIndex, (short) 0, size, sdoBuffer, soemConstants.EC_TIMEOUTRXM);
      sdoBuffer.position(0);
      sdoBuffer.limit(size);
      return wc;
   }
   
   
   public byte readSDOByte(int index, int subIndex)
   {
      readSDOToBuffer(index, subIndex, 1, sdoBuffer);
      return sdoBuffer.get();
   }
   public int readSDOUnsignedByte(int index, int subIndex)
   {
      return readSDOByte(index, subIndex) & 0xFF;
   }

   public short readSDOShort(int index, int subIndex)
   {
      readSDOToBuffer(index, subIndex, 2, sdoBuffer);
      return sdoBuffer.getShort();
   }
   
   public int readSDOUnsignedShort(int index, int subIndex)
   {
      return readSDOShort(index, subIndex) & 0xFFFF;
   }
   
   public int readSDOInt(int index, int subIndex)
   {
      readSDOToBuffer(index, subIndex, 4, sdoBuffer);
      return sdoBuffer.getInt();
   }
   
   public long readSDOUnsignedInt(int index, int subIndex)
   {
      return readSDOInt(index, subIndex) & 0xFFFFFFFFl;
   }
   
   public long readSDOLong(int index, int subIndex)
   {
      readSDOToBuffer(index, subIndex, 8, sdoBuffer);
      return sdoBuffer.getLong();
   }
   
   public float readSDOFloat(int index, int subIndex)
   {
      readSDOToBuffer(index, subIndex, 4, sdoBuffer);
      return sdoBuffer.getFloat();
   }
   
   public double readSDODouble(int index, int subIndex)
   {
      readSDOToBuffer(index, subIndex, 8, sdoBuffer);
      return sdoBuffer.getDouble();
   }
   
   
   
   
   
   public void configureDCSync0(boolean enable, long sync0time, int sync0shift)
   {
      soem.ecx_dcsync0(context, slaveIndex, enable?(short)1:(short)0, sync0time, sync0shift);
   }
   
   public int processDataSize()
   {
      int size = 0;
      for(int i = 0; i < syncManagers.length; i++)
      {
         if(syncManagers[i] != null)
         {
            size += syncManagers[i].processDataSize();
         }
      }
      
      return size;
   }

   public int getAliasAddress()
   {
      return aliasAddress;
   }

   public int getConfigAddress()
   {
      return configAddress;
   }

   @Override
   public String toString()
   {
      return "Slave [aliasAddress=" + aliasAddress + ", configAddress=" + configAddress + "]";
   }
   
   public void linkBuffers(ByteBuffer ioMap)
   {
      int inputOffset =  soem.ecx_inputoffset(ec_slave, ioMap);
            
      for(int i = 0; i < syncManagers.length; i++)
      {
         if(syncManagers[i].getMailbusDirection() == MailbusDirection.TXPDO)
         {
            inputOffset += syncManagers[i].linkBuffers(ioMap, inputOffset);
         }
      }

      int outputOffset =  soem.ecx_outputoffset(ec_slave, ioMap);
      for(int i = 0; i < syncManagers.length; i++)
      {
         if(syncManagers[i].getMailbusDirection() == MailbusDirection.RXPDO)
         {
            outputOffset += syncManagers[i].linkBuffers(ioMap, outputOffset);
         }
      }
   }
   
   public int getALStatusCode()
   {
      return ec_slave.getALstatuscode();
   }
   
   public String getALStatusMessage()
   {
      return soem.ec_ALstatuscode2string(getALStatusCode());
   }
   
   public SyncManager syncManager(int index)
   {
      return syncManagers[index];
   }
   
   public SyncManager sm(int index)
   {
      return syncManager(index);
   }

   public boolean isOperational()
   {
      return true;
   }

   public int getState()
   {
      
      return ec_slave.getState();
   }

   
}
