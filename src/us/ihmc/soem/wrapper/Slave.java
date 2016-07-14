package us.ihmc.soem.wrapper;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import us.ihmc.soem.generated.ec_slavet;
import us.ihmc.soem.generated.ec_state;
import us.ihmc.soem.generated.ecx_contextt;
import us.ihmc.soem.generated.soem;
import us.ihmc.soem.generated.soemConstants;
import us.ihmc.soem.wrapper.SyncManager.MailbusDirection;

/**
 * EtherCAT slave description. Base class for all EtherCAT slaves
 * 
 * @author Jesper Smith
 *
 */
public abstract class Slave
{
   public static final int MAX_DC_OFFSET = 1000;
   public static final int MAX_DC_OFFSET_SAMLES = 10;

   /**
    * EtherCAT state machine states.
    *
    */
   public enum State
   {
      BOOT, INIT, PRE_OP, PRE_OPERR, SAFE_OP, SAFE_OPERR, OP, OFFLINE
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
   private boolean dcEnabled;

   private volatile State state = State.OFFLINE;
   
   
   
   /**
    * Create a new slave and set the address 
    * 
    * @param aliasAddress The address of the slave.
    * @param configAddress Position relative to aliasAddress
    */
   public Slave(int aliasAddress, int configAddress)
   {
      this.aliasAddress = aliasAddress;
      this.configAddress = configAddress;

      sdoBuffer.order(ByteOrder.nativeOrder()); // EtherCAT is a big-endian protocol, but this has to be LITTLE_ENDIAN

      dcDiff.order(ByteOrder.nativeOrder());
   }

   /**
    * Internal function. Configures the slave before the switch from pre-op to safe-op. Sets up the PDO assignment.
    * 
    * @param context
    * @param slave
    * @param slaveIndex
    */
   
   void configure(ecx_contextt context, ec_slavet slave, int slaveIndex, boolean enableDC, long cycleTimeInNs)
   {
      this.context = context;
      this.ec_slave = slave;
      this.slaveIndex = slaveIndex;

      Master.trace("Configuring DC settings");
      configure(enableDC, cycleTimeInNs);


      for (int i = 0; i < syncManagers.length; i++)
      {
         if (syncManagers[i] != null)
         {
            Master.trace("Configuring Sync Manager " + i);
            syncManagers[i].configure(this);
         }
      }

      Master.trace("Checking start bit offsets");
      if (ec_slave.getIstartbit() != 0 || ec_slave.getOstartbit() != 0)
      {
         throw new RuntimeException("Cannot configure slaves with non-zero start bits. Current slave is " + slave.getName());
      }

   }

   /**
    * Internal function.
    * 
    * @param isDCMasterClock
    */
   void setDCMasterClock(boolean isDCMasterClock)
   {
      this.dcMasterClock = isDCMasterClock;
   }

   /**
    * Internal function. Amount of bytes input data takes according to the physical slave configuration.
    * 
    * @return the amount of bytes the input data buffer needs
    */
   long getInputBytes()
   {
      return ec_slave.getIbytes();
   }

   /**
    * Internal function. Amount of bytes output data takes according to the physical slave configuration.
    * 
    * @return the amount of bytes the output data buffer needs
    */
   long getOutputBytes()
   {
      return ec_slave.getObytes();
   }

   /**
    * Registers a SyncManager. 
    * 
    * Every slave has 4 sync managers, but only the used ones have to be registered.  
    * 
    * @param syncManager
    */
   public void registerSyncManager(SyncManager syncManager)
   {
      if (syncManagers[syncManager.getIndex()] != null)
      {
         throw new RuntimeException("SyncManager " + syncManager.getIndex() + " has already been registered");
      }

      syncManagers[syncManager.getIndex()] = syncManager;
   }

   /**
    * Write the data in buffer from position 0 to buffer.position() to a SDO index. Blocking.
    * 
    *  Do not use in cyclical operation, register WriteSDO objects with the master to avoid blocking.
    * 
    * @param index Index of the SDO 
    * @param subindex Subindex of the SDO
    * @param buffer data
    * @return working counter
    */
   public int writeSDO(int index, int subindex, ByteBuffer buffer)
   {
      return soem.ecx_SDOwrite(context, slaveIndex, index, (short) subindex, (short) 9, buffer.position(), buffer, soemConstants.EC_TIMEOUTRXM);
   }

   
   /**
    * Writes a double to a SDO index. Blocking.
    * 
    *  Do not use in cyclical operation, register WriteSDO objects with the master to avoid blocking.
    * 
    * @param index Index of the SDO 
    * @param subindex Subindex of the SDO
    * @param value data
    * @return working counter
    */
   public int writeSDO(int index, int subIndex, double value)
   {
      sdoBuffer.clear();
      sdoBuffer.putDouble(0, value);
      return writeSDO(index, subIndex, sdoBuffer);
   }
   
   /**
    * Writes a long to a SDO index. Blocking.
    * 
    *  Do not use in cyclical operation, register WriteSDO objects with the master to avoid blocking.
    * 
    * @param index Index of the SDO 
    * @param subindex Subindex of the SDO
    * @param value data
    * @return working counter
    */
   public int writeSDO(int index, int subIndex, long value)
   {
      sdoBuffer.clear();
      sdoBuffer.putLong(0, value);
      return writeSDO(index, subIndex, sdoBuffer);
   }
   
   /**
    * Writes a float to a SDO index. Blocking.
    * 
    *  Do not use in cyclical operation, register WriteSDO objects with the master to avoid blocking.
    * 
    * @param index Index of the SDO 
    * @param subindex Subindex of the SDO
    * @param value data
    * @return working counter
    */
   public int writeSDO(int index, int subIndex, float value)
   {
      sdoBuffer.clear();
      sdoBuffer.putFloat(0, value);
      return writeSDO(index, subIndex, sdoBuffer);
   }
   
   /**
    * Writes a int to a SDO index. Blocking.
    * 
    *  Do not use in cyclical operation, register WriteSDO objects with the master to avoid blocking.
    * 
    * @param index Index of the SDO 
    * @param subindex Subindex of the SDO
    * @param value data
    * @return working counter
    */
   public int writeSDO(int index, int subIndex, int value)
   {
      sdoBuffer.clear();
      sdoBuffer.putInt(0, value);
      return writeSDO(index, subIndex, sdoBuffer);
   }
   
   /**
    * Writes a short to a SDO index. Blocking.
    * 
    *  Do not use in cyclical operation, register WriteSDO objects with the master to avoid blocking.
    * 
    * @param index Index of the SDO 
    * @param subindex Subindex of the SDO
    * @param value data
    * @return working counter
    */
   public int writeSDO(int index, int subIndex, short value)
   {
      sdoBuffer.clear();
      sdoBuffer.putShort(0, value);
      return writeSDO(index, subIndex, sdoBuffer);
   }
   
   /**
    * Writes a byte to a SDO index. Blocking.
    * 
    *  Do not use in cyclical operation, register WriteSDO objects with the master to avoid blocking.
    * 
    * @param index Index of the SDO 
    * @param subindex Subindex of the SDO
    * @param value data
    * @return working counter
    */
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
    * @param index Index of the SDO
    * @param subIndex Subindex of the SDO
    * @param size Size of the SDO data in bytes
    * @param sdoBuffer buffer to store data, has to be at least size bytes
    * 
    * @return working counter
    */
   public int readSDOToBuffer(int index, int subIndex, int size, ByteBuffer sdoBuffer)
   {
      if(sdoBuffer.capacity() < size)
      {
         throw new RuntimeException("Cannot read data of size " + size + "bytes into buffer. Increase buffer size");
      }
      int wc = soem.ecx_SDOread_java_helper(context, slaveIndex, index, (short) subIndex, (short) 0, size, sdoBuffer, soemConstants.EC_TIMEOUTRXM);
      sdoBuffer.position(0);
      sdoBuffer.limit(size);
      return wc;
   }
   
   /**
    * Read SDO with data of type byte (int8)
    * 
    * @param index Index of the SDO
    * @param subIndex Subindex of the SDO
    * 
    * @return SDO value
    */
   public byte readSDOByte(int index, int subIndex)
   {
      readSDOToBuffer(index, subIndex, 1, sdoBuffer);
      return sdoBuffer.get();
   }
   
   /**
    * Read SDO with data of type unsigned byte (uint8)
    * 
    * @param index Index of the SDO
    * @param subIndex Subindex of the SDO
    * 
    * @return SDO value
    */
   public int readSDOUnsignedByte(int index, int subIndex)
   {
      return readSDOByte(index, subIndex) & 0xFF;
   }

   
   /**
    * Read SDO with data of type short (int16).
    * 
    * @param index Index of the SDO
    * @param subIndex Subindex of the SDO
    * 
    * @return SDO value
    */
   public short readSDOShort(int index, int subIndex)
   {
      readSDOToBuffer(index, subIndex, 2, sdoBuffer);
      return sdoBuffer.getShort();
   }

   /**
    * Read SDO with data of type unsigned short (uint16).
    * 
    * @param index Index of the SDO
    * @param subIndex Subindex of the SDO
    * 
    * @return SDO value
    */
   public int readSDOUnsignedShort(int index, int subIndex)
   {
      return readSDOShort(index, subIndex) & 0xFFFF;
   }

   
   /**
    * Read SDO with data of type int (int32).
    * 
    * @param index Index of the SDO
    * @param subIndex Subindex of the SDO
    * 
    * @return SDO value
    */
   public int readSDOInt(int index, int subIndex)
   {
      readSDOToBuffer(index, subIndex, 4, sdoBuffer);
      return sdoBuffer.getInt();
   }

   /**
    * Read SDO with data of type unsigned int (uint32).
    * 
    * @param index Index of the SDO
    * @param subIndex Subindex of the SDO
    * 
    * @return SDO value
    */
   public long readSDOUnsignedInt(int index, int subIndex)
   {
      return readSDOInt(index, subIndex) & 0xFFFFFFFFl;
   }

   /**
    * Read SDO with data of type long (int64).
    * 
    * @param index Index of the SDO
    * @param subIndex Subindex of the SDO
    * 
    * @return SDO value
    */
   public long readSDOLong(int index, int subIndex)
   {
      readSDOToBuffer(index, subIndex, 8, sdoBuffer);
      return sdoBuffer.getLong();
   }

   /**
    * Read SDO with data of type float (32 bit).
    * 
    * @param index Index of the SDO
    * @param subIndex Subindex of the SDO
    * 
    * @return SDO value
    */
   public float readSDOFloat(int index, int subIndex)
   {
      readSDOToBuffer(index, subIndex, 4, sdoBuffer);
      return sdoBuffer.getFloat();
   }

   /**
    * Read SDO with data of type double (64 bit).
    * 
    * @param index Index of the SDO
    * @param subIndex Subindex of the SDO
    * 
    * @return SDO value
    */
   public double readSDODouble(int index, int subIndex)
   {
      readSDOToBuffer(index, subIndex, 8, sdoBuffer);
      return sdoBuffer.getDouble();
   }

   /**
    * Configure the Sync 0 pulse of this slave 
    * 
    * Use only sync 0 if the AssignActive word in the XML description is #0400
    * 
    * @param enable enable or disable DC
    * @param sync0time time between pulses
    * @param sync0shift shift with respect to the control thread
    */
   protected void configureDCSync0(boolean enable, long sync0time, int sync0shift)
   {
      soem.ecx_dcsync0(context, slaveIndex, enable?(short) 1 :(short)0, sync0time, sync0shift);
      this.dcEnabled = enable;
   }
   

   /**
    * Configure the Sync 0  and Sycn 1 pulse of this slave 
    * 
    * Use sync 0 and sync 1 if the AssignActive word in the XML description is #0700
    * 
    * @param enable enable or disable DC
    * @param sync0time time between pulses
    * @param sync1time offset of the sync 1 pulse with respect to the sync0 pulse
    * @param sync0shift shift with respect to the control thread
    */
   protected void configureDCSync01(boolean enable, long sync0time, long sync1time, int sync0shift)
   {
      soem.ecx_dcsync01(context, slaveIndex, enable?(short) 1 :(short)0, sync0time, sync1time, sync0shift);
      this.dcEnabled = enable;
   }

   /**
    * Internal function. Gets the size of the underlying databuffer this slave needs, calculated from the configured PDO's. Should be getInputBytes() + getOutputBytes()
    * 
    * @return process data size
    */
   int processDataSize()
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

   /**
    * 
    * @return Alias address
    */
   public int getAliasAddress()
   {
      return aliasAddress;
   }

   /**
    * 
    * @return Ring position
    */
   public int getConfigAddress()
   {
      return configAddress;
   }

   @Override
   public String toString()
   {
      return "Slave [aliasAddress=" + aliasAddress + ", configAddress=" + configAddress + "]";
   }

   /**
    * Internal function, configures the syncmanagers and PDO data to backing memory storage.
    * 
    * @param ioMap
    */
   void linkBuffers(ByteBuffer ioMap)
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

   /**
    * Get the AL status code. The AL status code is useful to debug problems switching to OP state
    * 
    * @return AL Status code
    */
   public int getALStatusCode()
   {
      return ec_slave.getALstatuscode();
   }

   /**
    * Get human readable version of the AL Status code.
    * 
    * @return AL Status message
    */
   public String getALStatusMessage()
   {
      return soem.ec_ALstatuscode2string(getALStatusCode());
   }

   /**
    * Convenience function to address a syncmanager
    * @param index
    * 
    * @return Sync Manager at index
    */
   protected SyncManager syncManager(int index)
   {
      return syncManagers[index];
   }

   /**
    * Convenience function to address a syncmanager
    * @param index
    * 
    * @return Sync Manager at index
    */
   protected SyncManager sm(int index)
   {
      return syncManager(index);
   }

   /**
    * 
    * Returns if the state is operational.
    * 
    * The internal state gets updated on the house holding thread. The state can be delayed by several cycles.
    * 
    * @return true if the state is Operational.
    */
   public boolean isOperational()
   {
      return getState() == State.OP;
   }


   /**
    * 
    * Returns the slave state.
    * 
    * The internal state gets updated on the house holding thread. The state can be delayed by several cycles.
    * 
    * @return Slave state
    */
   public State getState()
   {
      return state;
   }

   private State requestState()
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
      else if (state == ec_state.EC_STATE_PRE_OP.swigValue() + ec_state.EC_STATE_ERROR.swigValue())
      {
         return State.PRE_OPERR;
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
    * Blocking. Sends out a datagram to read the ECT_REG_DCSYSDIFF to read the maximum difference between the internal slave clock and the master clock
    */
   private int getDCSyncOffset()
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

   
   /**
    * Internal function. Householding functionality. Gets called cyclically by the householding thread.
    * 
    * This function does the state control of the slave.
    * 
    */
   void doEtherCATStateControl()
   {

      this.state = requestState();

      switch (this.state)
      {
      case BOOT:
         break;
      case INIT:
         break;
      case PRE_OP:
         dcOffsetSamples = 0;
         break;
      case PRE_OPERR:
         System.out.println(toString() + " " +  soem.ec_ALstatuscode2string(ec_slave.getALstatuscode()));
         break;
      case SAFE_OP:
         if (dcEnabled)
         {
            int dcOffset = getDCSyncOffset();
            if (dcOffset < MAX_DC_OFFSET && dcOffset > -MAX_DC_OFFSET)
            {
               // At boot dcOffset can be zero. Ignore this. Except for the master clock, which is always zero.
               if (dcOffset != 0 || dcMasterClock)
               {
                  if (dcOffsetSamples++ > MAX_DC_OFFSET_SAMLES)
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
         }
         else
         {
            System.out.println("Slave " + slaveIndex + " has DC disabled, switching to OP");
            ec_slave.setState(ec_state.EC_STATE_OPERATIONAL.swigValue());
            soem.ecx_writestate(context, slaveIndex);
         }
         break;
      case SAFE_OPERR:
         System.out.println(toString() + " " +  soem.ec_ALstatuscode2string(ec_slave.getALstatuscode()));
         dcOffsetSamples = 0;
         break;
      case OP:
         dcOffsetSamples = 0;
         break;
      case OFFLINE:         
         break;
      }

   }
   
   /**
    * Shutdown hook for the slave. Use to clear up state machines
    * 
    * Will be called cyclically on shutdown till hasShutDown is true.
    */
   protected abstract void shutdown();
   
   /**
    * Signal that the slave has succesfully been shutdown. 
    * 
    * Always returning true is valid and will result in skipping the shutdown() procedure for this slave.
    * 
    * @return true if the slave has shutdown
    * 
    */
   protected abstract boolean hasShutdown();

   
   /**
    * Callback method when configuring the slave. Use to set required SDOS and configure the correct DC settings.
    * 
    * If no DC is desired, call configureSync0(false,0,0)
    * If only sync0 is desired (assignActivate = #0400) call configureDCSync0(true, cycleTimeInNs, shift0)
    * If both sync0 and sync 1 are desired (assignActivate = #0700) call configureDCSync01
    * 
    * @param cycleTimeInNs
    */
   protected abstract void configure(boolean dcEnabled, long cycleTimeInNs);


}
