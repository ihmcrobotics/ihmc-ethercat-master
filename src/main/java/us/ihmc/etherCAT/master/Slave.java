package us.ihmc.etherCAT.master;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

import us.ihmc.etherCAT.master.EtherCATStatusCallback.TRACE_EVENT;
import us.ihmc.etherCAT.master.SyncManager.MailbusDirection;
import us.ihmc.soem.generated.ec_slavet;
import us.ihmc.soem.generated.ec_state;
import us.ihmc.soem.generated.ecx_context;
import us.ihmc.soem.generated.ecx_portt;
import us.ihmc.soem.generated.soem;
import us.ihmc.soem.generated.soemConstants;

/**
 * EtherCAT slave description. Base class for all EtherCAT slaves
 * 
 * @author Jesper Smith
 *
 */
public class Slave
{
   public static final int MAX_DC_OFFSET_DEFAULT = 200;
   public static final int MAX_DC_OFFSET_SAMLES = 10;

   public static final int ECT_SMT_SIZE = 8;
   public static final int ECT_REG_WATCHDOG_DIV =  0x0400;
   public static final int ECT_REG_WATCHDOG_PDO_TIMEOUT = 0x0420;
   
   /**
    * EtherCAT state machine states.
    *
    * The states are ordered from Offline to OP.
    */
   public enum State
   {
      OFFLINE, BOOT, INIT, PRE_OP, PRE_OPERR, SAFE_OP, SAFE_OPERR, OP
   }

   private final int aliasAddress;
   private final int position;
   private final long vendor;
   private final long productCode;
   private final ByteBuffer sdoBuffer = ByteBuffer.allocateDirect(128);

   private final SyncManager[] syncManagers = new SyncManager[4];

   private Master master;
   private ecx_context context;
   private ecx_portt port;
   private ec_slavet ec_slave;
   private int slaveIndex;
   private final ByteBuffer dcDiff = ByteBuffer.allocateDirect(4);
   
   private int maximumDCOffset = MAX_DC_OFFSET_DEFAULT;
   private int dcOffsetSamples = 0;
   private boolean dcEnabled;

   
   private State houseHolderState = State.OFFLINE;
   private State state = State.OFFLINE; 
   
   private int houseHolderAlStatusCode = 0;
   private int alStatusCode = 0;
   
   private long cycleTimeInNs;
   
   private final ArrayList<SDO> SDOs = new ArrayList<>();
   
   private boolean configurePDOWatchdog = false;
   private int pdoWatchdogTimeout = 0;
   
   private boolean dcClockStable = false;
   
   
   private final ByteBuffer alStateBuffer = ByteBuffer.allocateDirect(3 * Short.BYTES);
   private final ByteBuffer rxErrorBuffer = ByteBuffer.allocateDirect(19 * Short.BYTES);
   
   
   private int[] rxFrameErrorCounter = new int[4];
   private int[] rxPhysicalLayerErrorCounter = new int[4];
   private int[] lostLinkCounter = new int[4];
   
   /**
    * Create a new slave and set the address 
    * 
    * Slaves are addressed based on their aliasAddress and their position relative to the previous set aliasAddress.
    * 
    * The alias can be changed with the "eepromtool" utility that comes with the SOEM master. Power cycle the slave after running.
    * 
    * - When the alias is set and unique on the ring, the aliasAddress is the same as the address and the position is 0
    * - When the alias is set and the same as the previous alias on the ring, the aliasAddress is the same as the address and the position is incremented by one.
    * - When the alias is not set or set to zero, the aliasAddress is inherted from the previous slave on the ring and the position is incremented by one.
    * - When the alias is set and the same as a non-adjacent slave on the ring, the slave configuration is considered invalid and the master will not start.
    * 
    * @param aliasAddress The address of the slave.
    * @param position Position relative to aliasAddress
    */
   public Slave(long vendor, long productCode, int aliasAddress, int position)
   {
      this.vendor = vendor;
      this.productCode = productCode;
      this.aliasAddress = aliasAddress;
      this.position = position;

      sdoBuffer.order(ByteOrder.LITTLE_ENDIAN); // EtherCAT is a LITTLE ENDIAN protocol
      dcDiff.order(ByteOrder.LITTLE_ENDIAN);
      alStateBuffer.order(ByteOrder.LITTLE_ENDIAN);
   }

   final long getVendor()
   {
      return vendor;
   }

   final long getProductCode()
   {
      return productCode;
   }

   
   public void configurePDOWatchdog(int watchdogTimeoutInNanoseconds)
   {
      if(ec_slave != null)
      {
         throw new RuntimeException("Cannot register PDOs after initializing the master");
      }
      
      configurePDOWatchdog = true;
      pdoWatchdogTimeout = watchdogTimeoutInNanoseconds;
   }

   /**
    * Internal function. Configures the slave before the switch from pre-op to safe-op. Sets up the PDO assignment.
    * 
    * @param context
    * @param slave
    * @param slaveIndex
    */
   
   void configure(Master master, ecx_context context, ecx_portt port, ec_slavet slave, int slaveIndex, boolean enableDC, long cycleTimeInNs)
   {
      this.master = master;
      this.context = context;
      this.port = port;
      this.ec_slave = slave;
      this.slaveIndex = slaveIndex;
      this.cycleTimeInNs = cycleTimeInNs;
      
      configureImpl(master, slave, enableDC, cycleTimeInNs);
   }

   private void configureImpl(Master master, ec_slavet slave, boolean enableDC, long cycleTimeInNs)
   {
      configureDCSync0(false, 0, 0);   // Disable DC Sync
      
      // Configure the PDO watchdog register by reading the divisor first
      if(configurePDOWatchdog)
      {
         ByteBuffer pdoWatchdogBuffer = ByteBuffer.allocateDirect(4);
         pdoWatchdogBuffer.order(ByteOrder.LITTLE_ENDIAN);
         
         master.getEtherCATStatusCallback().trace(this, TRACE_EVENT.READ_WATCHDOG_DIV);
         int wc = soem.ecx_FPRD(port, ec_slave.getConfigadr(), ECT_REG_WATCHDOG_DIV, 4, pdoWatchdogBuffer, soemConstants.EC_TIMEOUTRET);
         if(wc > 0)
         {
            int watchdogDivRaw = pdoWatchdogBuffer.getInt(0);
            int watchdogDiv = 40 * (watchdogDivRaw + 2);
            
            int watchdogPDORaw = pdoWatchdogTimeout / watchdogDiv;
            pdoWatchdogBuffer.putInt(watchdogPDORaw);
            master.getEtherCATStatusCallback().notifyWatchdogConfiguration(this, pdoWatchdogTimeout, watchdogDiv, watchdogPDORaw);
            wc = soem.ecx_FPWR(port, ec_slave.getConfigadr(), ECT_REG_WATCHDOG_PDO_TIMEOUT, 4, pdoWatchdogBuffer, soemConstants.EC_TIMEOUTRET);
            if(wc == 0)
            {
               master.getEtherCATStatusCallback().notifyWatchdogConfigurationError(this);
            }
         }
         else
         {
            master.getEtherCATStatusCallback().notifyWatchdogConfigurationError(this);
         }
         
      }
      
      for (int i = 0; i < syncManagers.length; i++)
      {
         if (syncManagers[i] != null)
         {
            syncManagers[i].configure(master, this);
         }
      }

      master.getEtherCATStatusCallback().trace(this, TRACE_EVENT.CONFIGURE_DC);
      configure(enableDC, cycleTimeInNs);
      
      // Slaves are in SAFE_OP when the master has been initialized
      state = State.SAFE_OP;
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
    * @param timeout in nanoseconds
    * @return working counter
    */
   public int writeSDO(int index, int subindex, ByteBuffer buffer, int timeout)
   {
      int wc = soem.ecx_SDOwrite(context, slaveIndex, index, (short) subindex, (short) 0, buffer.position(), buffer, timeout);
      master.getEtherCATStatusCallback().notifySDOWrite(this, index, subindex, wc, buffer);
      return wc;
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
      return writeSDO(index, subindex, buffer, soemConstants.EC_TIMEOUTRXM);
   }
   
   
   /**
    * Write ASCII or UTF-8 string to SDO. 
    * 
    * Will reuse internal buffers to write the value if the string length < 128 characters. To 
    * improve performance, conversion to ASCII is done using casting of the string characters. UTF-16
    * characters will fail.
    * 
    * @param index Index of the SDO 
    * @param subindex Subindex of the SDO
    * @param string data to write, behavior unspecified if not all characters are ASCII
    * @return working counter
    */
   public int writeSDOASCII(int index, int subindex, String string)
   {
      return writeSDOASCII(index, subindex, string, soemConstants.EC_TIMEOUTRXM);
   }
   
   /**
    * Write ASCII or UTF-8 string to SDO. 
    * 
    * Will reuse internal buffers to write the value if the string length < 128 characters. To 
    * improve performance, conversion to ASCII is done using casting of the string characters. UTF-16
    * characters will fail.
    * 
    * @param index Index of the SDO 
    * @param subindex Subindex of the SDO
    * @param string data to write, behavior unspecified if not all characters are ASCII
    * @param timeout timeout in nanoseconds
    * @return working counter
    */
   public int writeSDOASCII(int index, int subindex, String string, int timeout)
   {
      ByteBuffer stringBuffer;
      if(string.length() > sdoBuffer.capacity())
      {
         stringBuffer = ByteBuffer.allocateDirect(string.length());
         stringBuffer.order(ByteOrder.LITTLE_ENDIAN);
      }
      else
      {
         stringBuffer = sdoBuffer;
      }
      stringBuffer.clear();
      
      for(int i = 0; i < string.length(); i++)
      {
         byte ch = (byte) string.charAt(i);
         stringBuffer.put(ch);
      }
      
      return writeSDO(index, subindex, stringBuffer, timeout);
   }
   

   /**
    * Write UTF-16 string to SDO. 
    * 
    * Will reuse internal buffers to write the value if the string length < 64 characters.
    * 
    * @param index Index of the SDO 
    * @param subindex Subindex of the SDO
    * @param string data to write
    * @param timeout timeout in nanoseconds
    * @return working counter
    */
   public int writeSDOUTF16(int index, int subindex, String string)
   {
      return writeSDOUTF16(index, subindex, string, soemConstants.EC_TIMEOUTRXM);
   }
   
   /**
    * Write UTF-16 string to SDO. 
    * 
    * Will reuse internal buffers to write the value if the string length < 64 characters.
    * 
    * @param index Index of the SDO 
    * @param subindex Subindex of the SDO
    * @param string data to write
    * @param timeout timeout in nanoseconds
    * @return working counter
    */
   public int writeSDOUTF16(int index, int subindex, String string, int timeout)
   {
      ByteBuffer stringBuffer;
      if(string.length() * 2 > sdoBuffer.capacity())
      {
         stringBuffer = ByteBuffer.allocateDirect(string.length() * 2);
         stringBuffer.order(ByteOrder.LITTLE_ENDIAN);
      }
      else
      {
         stringBuffer = sdoBuffer;
      }
      stringBuffer.clear();
      
      for(int i = 0; i < string.length(); i++)
      {
         char ch = string.charAt(i);
         stringBuffer.putChar(ch);
      }
      
      return writeSDO(index, subindex, stringBuffer, timeout);
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
      sdoBuffer.putDouble(value);
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
      sdoBuffer.putLong(value);
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
      sdoBuffer.putFloat(value);
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
      sdoBuffer.putInt(value);
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
      sdoBuffer.putShort(value);
      return writeSDO(index, subIndex, sdoBuffer);
   }
   
   /**
    * Writes a byte to a SDO index. Blocking.
    * 
    *  Do not use in cyclical operation, register WriteSDO objects with the master to avoid blocking.
    * 
    * @param index Index of the SDO 
    * @param subIndex Subindex of the SDO
    * @param value data
    * @return working counter
    */
   public int writeSDO(int index, int subIndex, byte value)
   {
      sdoBuffer.clear();
      sdoBuffer.put(value);
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
      master.getEtherCATStatusCallback().notifyReadSDO(this, index, size, subIndex, wc, sdoBuffer);
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
    * Internal function. Gets the size of the underlying databuffer this slave needs, calculated from the configured PDO's.
    * 
    * This function rounds up to the nearest byte. If the slave needs 4 bits, it will return 1 byte. This way, it is guaranteed that a large enough
    * buffer is allocated.
    * 
    * @return process data size in bytes, rounded up
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
   public int getPosition()
   {
      return position;
   }

   public String getName()
   {
      return getClass().getSimpleName();
   }
   
   @Override
   public String toString()
   {
      return "Slave [name=" + getName() + ", aliasAddress=" + aliasAddress + ", position=" + position + "]";
   }

   /**
    * Internal function, configures the syncmanagers and PDO data to backing memory storage.
    * 
    * @param ioMap
    * @throws IOException 
    */
   void linkBuffers(ByteBuffer ioMap) throws IOException
   {

      try
      {
         master.getEtherCATStatusCallback().notifySlaveBuffer(this, soem.ecx_inputoffset(ec_slave, ioMap), ec_slave.getIbits(), ec_slave.getIstartbit(), soem.ecx_outputoffset(ec_slave, ioMap), ec_slave.getObits(), ec_slave.getOstartbit());
         BufferOffsetHolder inputOffset = new BufferOffsetHolder(soem.ecx_inputoffset(ec_slave, ioMap), ec_slave.getIstartbit(), ec_slave.getIbits());
         
         for (int i = 0; i < syncManagers.length; i++)
         {
            if (syncManagers[i] != null && syncManagers[i].getMailbusDirection() == MailbusDirection.TXPDO)
            {
               syncManagers[i].linkBuffers(ioMap, inputOffset);
            }
         }
   
         
         if(inputOffset.getAvailableBits() != 0)
         {
            throw new IOException(toString() + ": " + inputOffset.getAvailableBits() + " unmapped bits in the input mappping. Make sure that your PDO configuration matches the slave information.");
         }
         
         BufferOffsetHolder outputOffset = new BufferOffsetHolder(soem.ecx_outputoffset(ec_slave, ioMap), ec_slave.getOstartbit(), ec_slave.getObits());
         for (int i = 0; i < syncManagers.length; i++)
         {
            if (syncManagers[i] != null && syncManagers[i].getMailbusDirection() == MailbusDirection.RXPDO)
            {
               syncManagers[i].linkBuffers(ioMap, outputOffset);
            }
         }
         if(outputOffset.getAvailableBits() != 0)
         {
            throw new IOException(toString() + ": " + outputOffset.getAvailableBits() + " unmapped bits in the output mappping. Make sure that your PDO configuration matches the slave information.");
         }
      }
      catch(IOException e)
      {
         throw new IOException(toString() + ": " + e.getMessage());
      }
   }

   /**
    * Get the AL status code. The AL status code is useful to debug problems switching to OP state
    * 
    * @return AL Status code
    */
   public int getALStatusCode()
   {
      return alStatusCode;
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

   private State getStateFromEcSlave(int state)
   {
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
    * Write register value to the slave.
    * 
    * Blocking low level function. Use with caution.
    * 
    * @param register Register to write to
    * @param value Value to write
    * @return true if successfull
    */
   public boolean writeRegister(int register, ByteBuffer value)
   {
      int wkc = soem.ecx_FPWR(port, ec_slave.getConfigadr(), register, value.remaining(), value, soemConstants.EC_TIMEOUTRET);
      if(wkc == soemConstants.EC_NOFRAME)
      {
         return false;
      }
      else
      {
         return true;
      }
   }

   /**
    * Blocking. Sends out a datagram to read the ECT_REG_DCSYSDIFF to read the maximum difference between the internal slave clock and the master clock
    */
   private int getDCSyncOffset()
   {

      int wkc = soem.ecx_FPRD(port, ec_slave.getConfigadr(), soemConstants.ECT_REG_DCSYSDIFF, 4, dcDiff, soemConstants.EC_TIMEOUTRET);
      if (wkc == soemConstants.EC_NOFRAME)
      {
         System.err.println("Cannot query DC Offset");
         return Integer.MAX_VALUE;
      }
      int diff = sm32ToInt32(dcDiff.getInt(0));
      return diff;
   }
   
   /**
    * Internal function. Update slave state from householding thread.
    */
   
   boolean updateEtherCATState()
   {
      
      int wc = soem.ecx_FPRD(port, ec_slave.getConfigadr(), soem.ECT_REG_ALSTAT, 3, alStateBuffer, soemConstants.EC_TIMEOUTRET);
      
      if(wc > 0)
      {
         State previousState = this.houseHolderState;
         short newState = alStateBuffer.getShort(0);
         this.ec_slave.setState(newState);
         this.houseHolderState = getStateFromEcSlave(newState);
         this.houseHolderAlStatusCode = alStateBuffer.getShort(2);
         if(previousState != this.houseHolderState)
         {
            master.getEtherCATStatusCallback().notifyStateChange(this, previousState, this.houseHolderState);
         }
         return true;
      }
      else
      {
         master.getEtherCATStatusCallback().notifyGetSlaveStateError(this);
         return false;
      }
   }
   
   /**
    * Internal function. Update RX Erros and link errors from householder thread
    * @return
    */
   boolean updateRXTXStats()
   {
      int wc = soem.ecx_FPRD(port, ec_slave.getConfigadr(), soem.ECT_REG_RXERR, rxErrorBuffer.capacity(), rxErrorBuffer, soemConstants.EC_TIMEOUTRET);
      
      if(wc > 0)
      {
         return true;
      }
      else
      {
         master.getEtherCATStatusCallback().notifyGetSlaveRXError(this);
         return false;
      }
   }
   

   public boolean clearRXErrors()
   {
      for(int i = 0; i < rxErrorBuffer.capacity(); i++)
      {
         rxErrorBuffer.put(i, (byte) -1);
      }
      
      int wc = soem.ecx_FPWR(port, ec_slave.getConfigadr(), soem.ECT_REG_RXERR, rxErrorBuffer.capacity(), rxErrorBuffer, soemConstants.EC_TIMEOUTRET);
      
      if(wc > 0)
      {
         master.getEtherCATStatusCallback().notifyClearSlaveRXErrorSuccess(this);
         return true;
      }
      else
      {
         master.getEtherCATStatusCallback().notifyClearSlaveRXErrorFailure(this);
         return false;
      }
   }
   
   /**
    * Internal function. Householding functionality. Gets called cyclically by the householding thread.
    * 
    * This function does the state control of the slave.
    * 
    */
   void doEtherCATStateControl(long runTime)
   {
      switch (this.houseHolderState)
      {
      case BOOT:
      case INIT:
         if(!dcEnabled)
         {
            master.getEtherCATStatusCallback().trace(this, TRACE_EVENT.RECONFIG_TO_PREOP);
            if(soem.ecx_reconfig_slave_to_preop(context, slaveIndex, soemConstants.EC_TIMEOUTRET3) > 0)
            {
               ec_slave.setIslost((short) 0);
            }
         }
         
         break;
      case PRE_OP:
         
         if(!dcEnabled)
         {
            master.getEtherCATStatusCallback().trace(this, TRACE_EVENT.RECONFIG_TO_SAFEOP);
            configureImpl(master, ec_slave, dcEnabled, cycleTimeInNs);
            if(soem.ecx_reconfig_slave_to_safeop(context, slaveIndex, soemConstants.EC_TIMEOUTRET3) > 0)
            {
               ec_slave.setIslost((short) 0);
            }
         }
         dcClockStable = false;
         dcOffsetSamples = 0;
         break;
      case PRE_OPERR:
         break;
      case SAFE_OP:
         if(dcEnabled && !dcClockStable)
         {
            int dcOffset = getDCSyncOffset();
            if (dcOffset < maximumDCOffset && dcOffset > -maximumDCOffset)
            {
               // At boot dcOffset can be zero. Ignore this. Except for the master clock, which is always zero.
               if (dcOffset != 0 || ec_slave.getDCprevious() == 0)
               {
                  if (dcOffsetSamples++ > MAX_DC_OFFSET_SAMLES)
                  {
                     dcClockStable = true;
                  }
               }
               else
               {
                  dcOffsetSamples = 0;
               }                     
            }
            else
            {
               master.getEtherCATStatusCallback().reportDCSyncWaitTime(this, runTime, dcOffset);               
            }
         }
         else
         {
            ec_slave.setState(ec_state.EC_STATE_OPERATIONAL.swigValue());
            soem.ecx_writestate(context, slaveIndex);
         }
         break;
      case SAFE_OPERR:
         dcOffsetSamples = 0;
         dcClockStable = false;
         ec_slave.setState(ec_state.EC_STATE_SAFE_OP.swigValue() + ec_state.EC_STATE_ACK.swigValue());
         soem.ecx_writestate(context, slaveIndex);
         break;
      case OP:
         break;
      case OFFLINE:         
         if(ec_slave.getIslost() == 0)
         {
            soem.ecx_statecheck(context, slaveIndex, ec_state.EC_STATE_OPERATIONAL.swigValue(), soemConstants.EC_TIMEOUTRET);
            if(ec_slave.getState() == 0)
            {
               ec_slave.setIslost((short)1);
               master.getEtherCATStatusCallback().trace(this, TRACE_EVENT.SLAVE_LOST);
            }
         }
         break;
      }
      
      if(!dcEnabled && ec_slave.getIslost() > 0)
      {
         if(ec_slave.getState() == 0)
         {
            master.getEtherCATStatusCallback().trace(this, TRACE_EVENT.RECOVER_SLAVE);
            if(soem.ecx_recover_slave(context, slaveIndex, soemConstants.EC_TIMEOUTRET3) > 0)
            {
               ec_slave.setIslost((short)0);
               master.getEtherCATStatusCallback().trace(this, TRACE_EVENT.RECOVERED_SLAVE);
            }
         }
         else
         {
            ec_slave.setIslost((short)0);
            master.getEtherCATStatusCallback().trace(this, TRACE_EVENT.SLAVE_FOUND);
         }
      }

   }
   
   
   
   /**
    * Shutdown hook for the slave. Use to clear up state machines
    * 
    * Will be called cyclically on shutdown till hasShutDown is true.
    */
   protected void shutdown()
   {
      
   }
   
   /**
    * Signal that the slave has succesfully been shutdown. 
    * 
    * Always returning true is valid and will result in skipping the shutdown() procedure for this slave.
    * 
    * @return true if the slave has shutdown
    * 
    */
   protected boolean hasShutdown()
   {
      return true;
   }

   
   /**
    * Callback method when configuring the slave. Use to set required SDOS and configure the correct DC settings.
    * 
    * If no DC is desired, call configureSync0(false,0,0)
    * If only sync0 is desired (assignActivate = #0400) call configureDCSync0(true, cycleTimeInNs, shift0)
    * If both sync0 and sync 1 are desired (assignActivate = #0700) call configureDCSync01
    * 
    * @param cycleTimeInNs
    */
   protected void configure(boolean dcEnabled, long cycleTimeInNs)
   {

      if (dcEnabled)
      {
         configureDCSync0(true, cycleTimeInNs, 0);
      }
      else
      {
         configureDCSync0(false, 0, 0);
      }
   }

   
   /**
    * Internal method to cleanup state
    */
   void cleanup()
   {
      configureDCSync0(false, 0, 0);   // Disable DC Sync
   }
   
   /**
    * Sets the maximum DC offset that is allowed before the slave is transfered from SAFEOP to OP. Only used when DC clocks are enabled.
    * 
    * Default is 500ns. Minimum that is probable to work is about 5ns.
    * 
    * @param dcOffsetInNs 
    */
   public void setMaximumDCOffsetForOPMode(int dcOffsetInNs)
   {
      this.maximumDCOffset = dcOffsetInNs;
   }

   /**
    * True if the slaves support Complete Access or not.
    * 
    * We have seen some slaves that claim to support CA but do not. Override this function and return false to avoid CA usage.
    * 
    * @return true if the slave supports CA
    */
   public boolean supportsCA()
   {
      return true;
   }

   /**
    * Internal function. Returns the slave index on the EtherCAT bus
    * 
    * @return slave index for EtherCAT
    */
   int getSlaveIndex()
   {
      return slaveIndex;
   }


   /**
    * Check if the slave is configured.
    * 
    * @return true if the slave is configured by the master
    */
   public boolean isConfigured()
   {
      return ec_slave != null;
   }
   
   /**
    * Register a SDO object before cyclic operation. The SDO object can request data without blocking from the control thread.
    * 
    * 
    * @param sdo SDO to register
    */
   public void registerSDO(SDO sdo)
   {
      if(sdo.getSlave() == this)
      {
         for(int i = 0; i < SDOs.size(); i++)
         {
            if(SDOs.get(i).equals(sdo))
            {
               throw new RuntimeException("Cannot register " + sdo + " twice.");
            }
         }
         
         SDOs.add(sdo);         
      }
      else
      {
         throw new RuntimeException("Cannot register " + sdo + " to " + toString());
      }
   }

   /**
    * Update state variables read in the householder thread
    */
   public void updateStateVariables()
   {
      this.state = this.houseHolderState;
      this.alStatusCode = this.houseHolderAlStatusCode;
            
      for(int i = 0; i < 4; i++)
      {
         rxFrameErrorCounter[i] = rxErrorBuffer.get(0x0 + 2 * i);
         rxPhysicalLayerErrorCounter[i] = rxErrorBuffer.get(0x1 + 2 * i);
         lostLinkCounter[i] = rxErrorBuffer.get(0x10 + i);
      }
      
      for(int i = 0; i < SDOs.size(); i++)
      {
         SDOs.get(i).syncDataWithStatemachineThread();
      }
   }
   
   
   State getHouseholderState()
   {
      return this.houseHolderState;
   }

   public ArrayList<SDO> getSDOs()
   {
      return SDOs;
   }
   
   /**
    * Return the number of ports for this device.
    * 
    * This will return 4 by default, as that is the maximum ports an EtherCAT device can have.
    * Can be overloaded to return the actual number of ports.
    * 
    * This could be useful for visualizing RX errors, and only showing the active ports instead of 4.
    * 
    * @return Number of ports for this device.
    */
   public int getNumberOfPorts()
   {
      return 4;
   }
   
   public int getRxFrameErrorCounter(int port)
   {
      return rxFrameErrorCounter[port];
   }
   
   public int getRxPhysicalLayerErrorCounter(int port)
   {
      return rxPhysicalLayerErrorCounter[port];
   }
   
   public int getLostLinkCounter(int port)
   {
      return lostLinkCounter[port];
   }

}
