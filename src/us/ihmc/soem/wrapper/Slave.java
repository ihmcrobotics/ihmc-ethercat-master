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
   private final ByteBuffer sdoBuffer = ByteBuffer.allocateDirect(8);

   private final SyncManager[] syncManagers = new SyncManager[4];
   
   private ecx_contextt context;
   private ec_slavet ec_slave;
   private int slaveIndex;

   public Slave(int aliasAddress, int configAddress)
   {
      this.aliasAddress = aliasAddress;
      this.configAddress = configAddress;

      sdoBuffer.order(ByteOrder.BIG_ENDIAN);
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
      
   }
   
   
   public void registerSyncManager(SyncManager syncManager)
   {
      if(syncManagers[syncManager.getIndex()] != null)
      {
         throw new RuntimeException("SyncManager " + syncManager.getIndex() + " has already been registered");
      }
      
      syncManagers[syncManager.getIndex()] = syncManager;
   }

   public void writeSDO(int index, int subIndex, double value)
   {
      sdoBuffer.putDouble(0, value);
      soem.ecx_SDOwrite(context, slaveIndex, index, (short) subIndex, (short) 0, 8, sdoBuffer, soemConstants.EC_TIMEOUTRXM);
   }

   public void writeSDO(int index, int subIndex, long value)
   {
      sdoBuffer.putLong(0, value);
      soem.ecx_SDOwrite(context, slaveIndex, index, (short) subIndex, (short) 0, 8, sdoBuffer, soemConstants.EC_TIMEOUTRXM);
   }
   
   public void writeSDO(int index, int subIndex, float value)
   {
      sdoBuffer.putFloat(0, value);
      soem.ecx_SDOwrite(context, slaveIndex, index, (short) subIndex, (short) 0, 4, sdoBuffer, soemConstants.EC_TIMEOUTRXM);
   }

   public void writeSDO(int index, int subIndex, int value)
   {
      sdoBuffer.putInt(0, value);
      soem.ecx_SDOwrite(context, slaveIndex, index, (short) subIndex, (short) 0, 4, sdoBuffer, soemConstants.EC_TIMEOUTRXM);
   }

   public void writeSDO(int index, int subIndex, short value)
   {
      sdoBuffer.putShort(0, value);
      soem.ecx_SDOwrite(context, slaveIndex, index, (short) subIndex, (short) 0, 2, sdoBuffer, soemConstants.EC_TIMEOUTRXM);
   }

   public void writeSDO(int index, int subIndex, byte value)
   {
      sdoBuffer.put(0, value);
      soem.ecx_SDOwrite(context, slaveIndex, index, (short) subIndex, (short) 0, 1, sdoBuffer, soemConstants.EC_TIMEOUTRXM);
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

}
