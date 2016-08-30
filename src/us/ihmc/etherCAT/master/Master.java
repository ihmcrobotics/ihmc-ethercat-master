package us.ihmc.etherCAT.master;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

import us.ihmc.etherCAT.master.EtherCATStatusCallback.TRACE_EVENT;
import us.ihmc.soem.generated.ec_slavet;
import us.ihmc.soem.generated.ec_state;
import us.ihmc.soem.generated.ecx_contextt;
import us.ihmc.soem.generated.soem;
import us.ihmc.soem.generated.soemConstants;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

/**
 * 
 * EtherCAT master class. Advanced use only, most developers want to use EtherCATRealtimeThread to develop their controllers.
 * 
 * This class will create a separate EtherCAT controller thread that does various non-realtime householding tasks for 
 * the EtherCAT protocol, like state switching and SDO communication.
 * 
 * 
 * @author Jesper Smith
 *
 */
public class Master implements MasterInterface
{
   public static final long MAXIMUM_EXECUTION_JITTER_DEFAULT = 25000;
   public static final boolean DISABLE_CA = true;
   
   static
   {
      NativeLibraryLoader.loadLibrary("us.ihmc.soem.generated", "soemJava");
   }
   
   private static boolean initialized = false;
   
   private EtherCATStatusCallback etherCATStatusCallback = new EtherCATStatusCallback(false);
   private final EtherCATStateMachine etherCATStateMachine = new EtherCATStateMachine(this);
   
   private final ArrayList<Slave> registeredSlaves = new ArrayList<>();
   private final String iface;
   
   private ecx_contextt context = null;
   private Slave[] slaveMap;
   
   private ByteBuffer ioMap;
   
   private boolean enableDC = false;
   private long cycleTimeInNs = -1;
   
   private long startTime;
   
   private int expectedWorkingCounter = 0;
   private volatile int actualWorkingCounter = 0;
   
   private long maximumExecutionJitter = MAXIMUM_EXECUTION_JITTER_DEFAULT;
   private long previousArrivalTime = 0;
   
   private volatile long jitterSamples = 0;
   private volatile long jitterEstimate = 0;
   
   private int ethercatReceiveTimeout = soemConstants.EC_TIMEOUTRET;
   
   private final ArrayList<SDO> sdos = new ArrayList<>();
   
   /**
    * Create new EtherCAT master.
    * 
    * This creates the master object, but does not initialize. Call init() before starting cyclic operation.
    * 
    * @param iface The network interface to listen on
    */
   public Master(String iface)
   {
      if(initialized)
      {
         throw new RuntimeException("Currently, only a single master instance is supported.");
      }
      
      this.iface = iface;
      initialized = true;
   }
   
   private Slave getSlave(int alias, int position) throws IOException
   {
      Slave retSlave = null;
      for(int i = 0; i < registeredSlaves.size(); i++)
      {
         Slave slave = registeredSlaves.get(i);
         
         if(slave.getAliasAddress() == alias && slave.getPosition() == position)
         {
            if(retSlave == null)
            {
               retSlave = slave;
            }
            else
            {
               throw new IOException("Cannot configure slave " + alias + ":" + position + ". Multiple registeredSlaves with this address are specified. Change the alias addresses to a unique value.");
            }
         }
      }
      return retSlave;
   }
   
   /**
    * Set a custom EtherCAT status handler.
    * 
    * Note that the calling thread of the status message is not guaranteed
    * 
    * @param callback Callback class for status messages from the master
    */
   public void setEtherCATStatusCallback(EtherCATStatusCallback callback)
   {
      this.etherCATStatusCallback = callback;
   }
   
   /**
    * Enable trace level debugging
    */
   public void enableTrace()
   {
      setEtherCATStatusCallback(new EtherCATStatusCallback(true));
   }
   
   /**
    * Internal function.
    * 
    * @return the EtherCAT status callback
    */
   EtherCATStatusCallback getEtherCATStatusCallback()
   {
      return this.etherCATStatusCallback;
   }
   
   /**
    * Register a SDO object before cyclic operation. The SDO object can request data without blocking from the control thread.
    * 
    * @param sdo
    */
   public void registerSDO(SDO sdo)
   {
      for(int i = 0; i < sdos.size(); i++)
      {
         if(sdos.get(i).equals(sdo))
         {
            throw new RuntimeException("Cannot register " + sdo + " twice.");
         }
      }
      sdos.add(sdo);
   }
   
   /**
    * Enable distributed clocks. Slaves will be configured for cyclic operation. 
    * 
    * Use DistributedClockRealtimeThread to synchronize the control thread to the Master Clock (slave 1) 
    * 
    * @param cycleTimeInNs Time in ns between each call to send()
    */
   public void enableDC(long cycleTimeInNs)
   {
      enableDC = true;
      this.cycleTimeInNs = cycleTimeInNs;
//      this.ethercatReceiveTimeout = (int) (cycleTimeInNs / 1000);
   }
   
   /**
    * Initialize the master, configure all registeredSlaves registered with registerSlave() 
    * 
    * On return, all registeredSlaves will be in SAFE_OP mode. Also, the EtherCAT house holding thread
    * will be started. 
    * 
    * If DC is enabled, the ethercat nodes will be switched to OP as soon as synchronization
    * is attained between the slave clocks and the master
    * 
    * If DC is disabled, the registeredSlaves will be switched to OP as soon as they come online.
    * 
    * @throws IOException
    */
   public void init() throws IOException
   {
      getEtherCATStatusCallback().trace(TRACE_EVENT.FAST_IRQ);
      setupFastIRQ(iface);

      getEtherCATStatusCallback().trace(TRACE_EVENT.CREATE_CONTEXT);
      context = soem.ecx_create_context();
      
      
      getEtherCATStatusCallback().trace(TRACE_EVENT.OPEN_INTERFACE);
      if(soem.ecx_init(context, iface) == 0)
      {
         throw new IOException("Cannot open interface " + iface + ". Make sure to run as root.");
      }
      
      
      
      
      getEtherCATStatusCallback().trace(TRACE_EVENT.INITIALIZING_SLAVES);
      if(soem.ecx_config_init(context, (short)0) == 0)
      {
         throw new IOException("Cannot initialize registeredSlaves");
      }
      
      if(enableDC)
      {
         boolean dcCapable = soem.ecx_configdc(context) == (short)1;
         enableDC = dcCapable;
         if(!enableDC)
         {
            getEtherCATStatusCallback().notifyDCNotCapable();
         }
      }

      if(enableDC)
      {
         getEtherCATStatusCallback().trace(TRACE_EVENT.DC_ENABLED);
      }
      else
      {
         getEtherCATStatusCallback().trace(TRACE_EVENT.DC_DISABLED);
      }
      
      
      getEtherCATStatusCallback().trace(TRACE_EVENT.CONFIGURING_SLAVES);
      int slavecount = soem.ecx_slavecount(context);
      if(registeredSlaves.size() != slavecount)
      {
         if(registeredSlaves.size() < slavecount)
         {
            throw new IOException("Not all registeredSlaves are configured, got " + slavecount + " registeredSlaves, expected " + registeredSlaves.size());
         }
         else
         {
            throw new IOException("Not all registeredSlaves are online, got " + slavecount + " registeredSlaves, expected " + registeredSlaves.size());
         }
      }
      
      slaveMap = new Slave[slavecount];
      int processDataSize = 0;
      
      int previousAlias = 0;
      int previousPosition = -1;
      for(int i = 0; i < slavecount; i++)
      {
         ec_slavet ec_slave = soem.ecx_slave(context, i + 1);
         
         int alias, position;
         if(ec_slave.getAliasadr() == 0 || ec_slave.getAliasadr() == previousAlias)
         {
            alias = previousAlias;
            position = previousPosition + 1;
         }
         else
         {
            alias = ec_slave.getAliasadr();
            position = 0;
         }
         
         
         Slave slave = getSlave(alias, position);
         
         
         if(slave != null)
         {
            if(slave.getVendor() != ec_slave.getEep_man() || slave.getProductCode() != ec_slave.getEep_id())
            {
               throw new IOException("Invalid slave configuration for slave " + slave.getAliasAddress() + ":" + slave.getPosition() + ". Invalid vendor and/or product code");
            }
            
            slave.configure(this, context, ec_slave, i + 1, enableDC, cycleTimeInNs);
            slaveMap[i] = slave;
            processDataSize += slave.processDataSize();
         }
         else
         {
            throw new IOException("Unconfigured slave on alias " + alias + ":" + position + ". Make sure to power cycle after changing alias addresses.");
         }
         
         // Disable Complete Access reading of SDO configuration.
         if(!slave.supportsCA() || DISABLE_CA)
         {
            short config = ec_slave.getCoEdetails();
            config &= ~soemConstants.ECT_COEDET_SDOCA;
            ec_slave.setCoEdetails(config);
         }
         
         previousAlias = alias;
         previousPosition = position;
         
      }
      getEtherCATStatusCallback().trace(TRACE_EVENT.ALLOCATE_IOMAP);
      

      
      
      ioMap = ByteBuffer.allocateDirect(processDataSize);
      ioMap.order(ByteOrder.LITTLE_ENDIAN);
      
      
      int ioBufferSize = soem.ecx_config_map_group(context, ioMap, (short)0);
      if(ioBufferSize != processDataSize)
      {
         throw new IOException("Cannot allocate memory for etherCAT I/O. Expected process size is " + processDataSize + ", allocated " + ioBufferSize);
      }      


      if(soem.ecx_statecheck(context, 0, ec_state.EC_STATE_SAFE_OP.swigValue(), soemConstants.EC_TIMEOUTSTATE) == 0)
      {
         throw new IOException("Cannot transfer to SAFE_OP state");
      }      
       
      getEtherCATStatusCallback().trace(TRACE_EVENT.LINK_BUFFERS);
      
      for(int i = 0; i < slavecount; i++)
      {
         Slave slave = slaveMap[i];
         slave.linkBuffers(ioMap);
      }
      
      getEtherCATStatusCallback().trace(TRACE_EVENT.CONFIGURE_TXRX);
      soem.ecx_send_processdata(context);
      soem.ecx_receive_processdata(context, soemConstants.EC_TIMEOUTRET);
      
      
      expectedWorkingCounter = context.getGrouplist().getOutputsWKC() * 2 + context.getGrouplist().getInputsWKC();
      getEtherCATStatusCallback().notifyExpectedWorkingCounter(expectedWorkingCounter);
            
      
      if(enableDC)
      {
         startTime = getDCTime();
      }
      etherCATStateMachine.setSlaves(slaveMap);
      
      
      getEtherCATStatusCallback().trace(TRACE_EVENT.CONFIGURE_COMPLETE);
   }
   
   /**
    * Internal function to speed up the EtherCAT send/receive time.  
    * 
    * Calls the equivalent of 
    * 
    * ethtool -C iface rx-usecs 0 rx-frames 1 tx-usecs 0 tx-frames 1
    * 
    * This drastically reduces the time to complete a send/receive cycle. Tests show a decrease from 200us to 50us.
    * 
    * @param iface
    * @throws IOException
    */
   private void setupFastIRQ(String iface) throws IOException
   {
      int ret = soem.ecx_setup_socket_fast_irq(iface);
      
      switch(ret)
      {
      case 10:
         System.err.println("Cannot setup fast IRQ settings on network card. OS is not Linux");
         break;
      case 70:
         throw new IOException("Cannot open control socket to setup network card. Make sure you are root.");
      case 76:
         System.err.println("Cannot read current coalesce options from network card");
         break;
      case 81:
         System.err.println("Cannot write desired coalesce options to network card");
         break;
      case 1:
         //sucess
         break;
      default:
         throw new IOException("Unknown return value " + ret + " while setting coalesce options");
         
      }
   }
   
   /**
    * Call cyclically to run the slave shutdown code. Does not call master.send()/master.receive()
    *  
    * @return true if all registeredSlaves are shut down.
    */
   public boolean shutdownSlaves()
   {
      boolean allSlavesShutdown = true;
      for(int i = 0; i < slaveMap.length; i++)
      {
         if(!slaveMap[i].hasShutdown())
         {
            allSlavesShutdown = false;
            slaveMap[i].shutdown();
         }
      }
      return allSlavesShutdown;
   }

   /**
    * Stop the EtherCAT master
    * 
    * This function will stop the house holder thread and bring all registeredSlaves to the INIT state.
    * 
    * Note: Calling the slave shutdown sequence is the responsibility of the caller. This function will only shutdown the master.
    * 
    */
   public void shutdown()
   {
      
      getEtherCATStatusCallback().trace(TRACE_EVENT.STOP_HOUSEHOLDER);
      etherCATStateMachine.shutDown();
   }
   
   /**
    * Send process data. Blocking. 
    * 
    * Call cyclically before receive().
    */
   public void send()
   {
      soem.ecx_send_processdata(context);
   }
   
   /**
    * Check if DC is enabled
    * @return true if DC is enabled after init(), false if not
    */
   public boolean getDCEnabled()
   {
      return enableDC;
   }

   /**
    * Read the process data. 
    * 
    * Call after send(). Check resulting value with master.getExpectedWorkingCounter()
    * 
    * @return working counter of soem.EC_NOFRAME (-1) if no datagram has been received
    */
   public int receive()
   {
      int wkc = soem.ecx_receive_processdata(context, ethercatReceiveTimeout);
      
      if(wkc == soem.EC_NOFRAME)
      {
         return wkc;
      }
      else
      {         
         if(enableDC)
         {
            // Calculate jitter using RFC 1889
            long arrivalTime = getDCTime();
            if(previousArrivalTime != 0)
            {
               long D = (arrivalTime - previousArrivalTime) - (cycleTimeInNs);
               if(D < 0) D = -D;
               
               long jitterEstimate = this.jitterEstimate; // Localize volatile variable         
               jitterEstimate += (D - jitterEstimate)/16;
               this.jitterEstimate = jitterEstimate;
               ++this.jitterSamples;
            }
            
            previousArrivalTime = arrivalTime;
         }
         
         this.actualWorkingCounter = wkc;
         return wkc;
      }
   }
   
   /**
    * Receive process data. Do not calculate jitter.
    * 
    * Use to send multiple datagrams per control cycle.
    * 
    * @return working counter or soem.EC_NOFRAME if datagram is lost
    */
   public int receiveSimple()
   {
      int wkc = soem.ecx_receive_processdata(context, ethercatReceiveTimeout);
      return wkc;
   }
   
   /**
    * Get the expected working counter from the master
    */
   public int getExpectedWorkingCounter()
   {
      return expectedWorkingCounter;
   }
   
   /**
    * Internal function. Get the actual working counter
    */
   int getActualWorkingCounter()
   {
      return actualWorkingCounter;
   }
   
   /**
    * Register a slave. 
    * 
    * The slave will be setup for cyclical operation when the master initializes. Call before init().
    * @param slave
    */
   public void registerSlave(Slave slave)
   {
      registeredSlaves.add(slave);
   }
   
   
   /**
    * Gets the time of the DC Master clock from the last datagram.
    * 
    * This value corresponds to when send() is called. More precisely, it is the moment the datagram
    * passed the DC master clock. Useful for synchronization between the control loop and the DC master clock.
    * 
    * @return distributed clock time in ns
    */
   public long getDCTime()
   {
      return soem.ecx_dcTime(context);
   }
   
   /**
    * Gets the time read from the DC Master clock from when init() was run
    * 
    * @return Time init() was run as seen from the Master Clock
    */
   public long getStartDCTime()
   {
      return startTime;
   }
   
   
   /**
    * Get an estimate of the jitter based on the time the EtherCAT datagram arrives at the DC Master Clock (usually, slave 1)
    * 
    * Calculated using the method described in RFC 1889
    * 
    * @return jitter estimate in nanoseconds, 0 when DC is disabled
    */
   public long getJitterEstimate()
   {
      return jitterEstimate;
   }
   
   /**
    * @return the number of samples the current jitter estimate is based on. 
    */
   public long getJitterSamples()
   {
      return jitterSamples;
   }
   
   /**
    * Set the absolute bound for the jitter estimate before allowing the registeredSlaves to go to OP mode. 
    * 
    * This depends on the precision of your realtime system and is only used when DC clocks are enabled. 
    * Setting this number too high could result in registeredSlaves refusing to go to OP mode.  
    * 
    * @param jitterInNanoseconds default 1000ns
    */
   public void setMaximumExecutionJitter(long jitterInNanoseconds)
   {
      this.maximumExecutionJitter = jitterInNanoseconds;
   }
   
   /**
    * Internal function
    * @return maximum execution jitter
    */
   long getMaximumExecutionJitter()
   {
      return maximumExecutionJitter;
   }
   
   /**
    * Internal function. Return the raw context
    * 
    * @return master context
    */
   ecx_contextt getContext()
   {
      return context;
   }
   
   /**
    * Call this function cyclically, either in a separate thread or after receive().
    * 
    * Make sure not to call this function concurrently with receive()
    */
   public void doEtherCATStateControl()
   {
      etherCATStateMachine.doStateControl();
   }
   
   
   /**
    * Gets the lowest EtherCAT state any slave is in
    * 
    * @return the minimum state the EtherCAT registeredSlaves are in
    */
   public Slave.State getState()
   {
      Slave.State minimum = Slave.State.OP;
      for(Slave slave : slaveMap)
      {
         if(slave.getState().ordinal() < minimum.ordinal())
         {
            minimum = slave.getState();
         }
      }
      return minimum;
   }
   
   /**
    * Disable recovery for offline or faulted states. 
    * 
    * Recommended for control systems where a loss of a state probaly has catastrophic consequences (for example, walking robots)
    */
   public void disableRecovery()
   {
      etherCATStateMachine.disableRecovery();
   }

   ArrayList<SDO> getSDOs()
   {
      return sdos;
   }
   
}
