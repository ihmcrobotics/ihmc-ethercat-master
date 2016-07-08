package us.ihmc.soem.wrapper;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import us.ihmc.soem.generated.ec_slavet;
import us.ihmc.soem.generated.ec_state;
import us.ihmc.soem.generated.ecx_contextt;
import us.ihmc.soem.generated.soem;
import us.ihmc.soem.generated.soemConstants;
import us.ihmc.soem.wrapper.SyncManager.MailbusDirection;

public class Slave
{
   public static final int MAX_DC_OFFSET = 1000;
   public static final int MAX_DC_OFFSET_SAMLES = 10;

   public enum State
   {
      BOOT, INIT, PRE_OP, SAFE_OP, SAFE_OPERR, OP, OFFLINE
   }

   private final int aliasAddress;
   private final int configAddress;
   private final ByteBuffer sdoBuffer = ByteBuffer.allocateDirect(12);

   private final SyncManager[] syncManagers = new SyncManager[4];

   private ecx_contextt context;
   private ec_slavet ec_slave;
   private int slaveIndex;
   private final ByteBuffer dcDiff = ByteBuffer.allocateDirect(4);
   
   private boolean dcMasterClock = false;
   private int dcOffsetSamples = 0;

   public Slave(int aliasAddress, int configAddress)
   {
      this.aliasAddress = aliasAddress;
      this.configAddress = configAddress;

      sdoBuffer.order(ByteOrder.nativeOrder()); // EtherCAT is a big-endian protocol, but this has to be LITTLE_ENDIAN

      dcDiff.order(ByteOrder.nativeOrder());
   }

   void configure(ecx_contextt context, ec_slavet slave, int slaveIndex)
   {
      this.context = context;
      this.ec_slave = slave;
      this.slaveIndex = slaveIndex;

      for (int i = 0; i < syncManagers.length; i++)
      {
         if (syncManagers[i] != null)
         {
            syncManagers[i].configure(this);
         }
      }

      if (ec_slave.getIstartbit() != 0 || ec_slave.getOstartbit() != 0)
      {
         throw new RuntimeException("Cannot configure slaves with non-zero start bits. Current slave is " + slave.getName());
      }

   }
   
   void setDCMasterClock(boolean isDCMasterClock)
   {
      this.dcMasterClock = isDCMasterClock;
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
      if (syncManagers[syncManager.getIndex()] != null)
      {
         throw new RuntimeException("SyncManager " + syncManager.getIndex() + " has already been registered");
      }

      syncManagers[syncManager.getIndex()] = syncManager;
   }

   public int writeSDO(int index, int subindex, ByteBuffer buffer)
   {
      return soem.ecx_SDOwrite(context, slaveIndex, index, (short) subindex, (short) 9, buffer.position(), buffer, soemConstants.EC_TIMEOUTRXM);
   }

   public int writeSDO(int index, int subIndex, double value)
   {
      sdoBuffer.clear();
      sdoBuffer.putDouble(0, value);
      return writeSDO(index, subIndex, sdoBuffer);
   }

   public int writeSDO(int index, int subIndex, long value)
   {
      sdoBuffer.clear();
      sdoBuffer.putLong(0, value);
      return writeSDO(index, subIndex, sdoBuffer);
   }

   public int writeSDO(int index, int subIndex, float value)
   {
      sdoBuffer.clear();
      sdoBuffer.putFloat(0, value);
      return writeSDO(index, subIndex, sdoBuffer);
   }

   public int writeSDO(int index, int subIndex, int value)
   {
      sdoBuffer.clear();
      sdoBuffer.putInt(0, value);
      return writeSDO(index, subIndex, sdoBuffer);
   }

   public int writeSDO(int index, int subIndex, short value)
   {
      sdoBuffer.clear();
      sdoBuffer.putShort(0, value);
      return writeSDO(index, subIndex, sdoBuffer);
   }

   public int writeSDO(int index, int subIndex, byte value)
   {
      sdoBuffer.clear();
      sdoBuffer.put(0, value);
      return writeSDO(index, subIndex, sdoBuffer);
   }

   /**
    * Read a SDO value to a buffer. 
    * 
    * Position and limit are set accordingly.
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
      soem.ecx_dcsync0(context, slaveIndex, enable ? (short) 1 : (short) 0, sync0time, sync0shift);
      System.out.println("DC ACTIVE" + ec_slave.getDCactive());
   }

   public int processDataSize()
   {
      int size = 0;
      for (int i = 0; i < syncManagers.length; i++)
      {
         if (syncManagers[i] != null)
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
      int inputOffset = soem.ecx_inputoffset(ec_slave, ioMap);

      for (int i = 0; i < syncManagers.length; i++)
      {
         if (syncManagers[i] != null && syncManagers[i].getMailbusDirection() == MailbusDirection.TXPDO)
         {
            inputOffset += syncManagers[i].linkBuffers(ioMap, inputOffset);
         }
      }

      int outputOffset = soem.ecx_outputoffset(ec_slave, ioMap);
      for (int i = 0; i < syncManagers.length; i++)
      {
         if (syncManagers[i] != null && syncManagers[i].getMailbusDirection() == MailbusDirection.RXPDO)
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

   public State getState()
   {
      int state = ec_slave.getState();
      if (state == ec_state.EC_STATE_BOOT.swigValue())
      {
         return State.BOOT;
      }
      else if (state == ec_state.EC_STATE_INIT.swigValue())
      {
         return State.INIT;
      }
      else if (state == ec_state.EC_STATE_PRE_OP.swigValue())
      {
         return State.PRE_OP;
      }
      else if (state == ec_state.EC_STATE_SAFE_OP.swigValue())
      {
         return State.SAFE_OP;
      }
      else if (state == ec_state.EC_STATE_SAFE_OP.swigValue() + ec_state.EC_STATE_ERROR.swigValue())
      {
         return State.SAFE_OPERR;
      }
      else if (state == ec_state.EC_STATE_OPERATIONAL.swigValue())
      {
         return State.OP;
      }
      else
      {
         return State.OFFLINE;
      }
   }

   int sm32ToInt32(int diff)
   {
      if (diff < 0)
      {
         diff = -(diff & 0x7FFFFFFF);
      }

      return diff;

   }

   /**
    * Blocking
    */
   public int getDCSyncOffset()
   {

      int wkc = soem.ecx_FPRD(context.getPort(), ec_slave.getConfigadr(), soemConstants.ECT_REG_DCSYSDIFF, 4, dcDiff, soemConstants.EC_TIMEOUTRET);
      if (wkc == soemConstants.EC_NOFRAME)
      {
         System.err.println("Cannot query DC Offset");
         return Integer.MAX_VALUE;
      }
      int diff = sm32ToInt32(dcDiff.getInt(0));
      return diff;
   }

   public void doEtherCATStateControl()
   {

      switch (getState())
      {
      case BOOT:
         break;
      case INIT:
         break;
      case PRE_OP:
         dcOffsetSamples = 0;
         break;
      case SAFE_OP:
         int dcOffset = getDCSyncOffset();
         if (dcOffset < MAX_DC_OFFSET && dcOffset > -MAX_DC_OFFSET)
         {
            // At boot dcOffset can be zero. Ignore this. Except for the master clock, which is always zero.
            if(dcOffset != 0 || dcMasterClock)  
            {
               
               System.out.println("DC OFFSET " + dcOffset);
               if(dcOffsetSamples++ > MAX_DC_OFFSET_SAMLES)
               {
                  ec_slave.setState(ec_state.EC_STATE_OPERATIONAL.swigValue());
                  soem.ecx_writestate(context, slaveIndex);
                  System.out.println("SWITCHING TO OP");
               }
               
            }
            else
            {
               dcOffsetSamples = 0;
            }

         }
         break;
      case SAFE_OPERR:
         dcOffsetSamples = 0;
         break;
      case OP:
         dcOffsetSamples = 0;
         if(!dcMasterClock)
         {
            System.out.println("DC OFFSET " + getDCSyncOffset());
         }
         break;
      case OFFLINE:
         break;
      }

   }

}
