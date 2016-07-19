package us.ihmc.soem.wrapper;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.concurrent.locks.LockSupport;

import us.ihmc.soem.generated.ec_slavet;
import us.ihmc.soem.generated.ec_state;
import us.ihmc.soem.generated.ecx_contextt;
import us.ihmc.soem.generated.soem;
import us.ihmc.soem.generated.soemConstants;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

/**
 * 
 * EtherCAT master class. 
 * 
 * This class is the main entry point for EtherCAT operations. It requires raw socket access, so it is recommended
 * to be run as root or administrator. 
 * 
 * This class will create a seperate EtherCAT controller thread that does various non-realtime householding tasks for 
 * the EtherCAT protocol, like state switching and SDO communication.
 * 
 * 
 * If you want (recommended) to use Distributed Clocks, use in conjunction with DistrubutedCLockRealtimeThread.
 * 
 * @author Jesper Smith
 *
 */
public class Master
{
   private static final boolean TRACE = false;
   
   static
   {
      NativeLibraryLoader.loadLibrary("us.ihmc.soem.generated", "soemJava");
   }
   
   private static boolean initialized = false;
   
   private final EtherCATMasterHouseHolder etherCATHouseHolder = new EtherCATMasterHouseHolder();
   
   private final ArrayList<Slave> slaves = new ArrayList<>();
   private final String iface;
   
   private ecx_contextt context = null;
   private Slave[] slaveMap;
   
   private ByteBuffer ioMap;
   
   private boolean enableDC = false;
   private long cycleTimeInNs = -1;
   
   private final EtherCATController etherCATController;
   
   /**
    * Create new EtherCAT master.
    * 
    * This creates the master object, but does not initialize. Call init() before starting cyclic operation.
    * 
    * @param iface The network interface to listen on
    */
   public Master(String iface, EtherCATController etherCATController)
   {
      if(initialized)
      {
         throw new RuntimeException("Currently, only a single master instance is supported.");
      }
      
      this.iface = iface;
      this.etherCATController = etherCATController;
      initialized = true;
   }
   
   private Slave getSlave(int alias, int position)
   {
      for(int i = 0; i < slaves.size(); i++)
      {
         Slave slave = slaves.get(i);
         
         if(slave.getAliasAddress() == alias && slave.getPosition() == position)
         {
            return slave;
         }
      }
      return null;
   }
   
   /**
    * Register a SDO object before cyclic operation. The SDO object can request data without blocking from the control thread.
    * 
    * @param sdo
    */
   public void registerSDO(SDO sdo)
   {
      etherCATHouseHolder.addSDO(sdo);
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
   }
   
   public static void trace(String msg)
   {
      if(TRACE)
      {
         System.out.println("[" + System.nanoTime() + "] " + msg);
      }
   }
   
   /**
    * Initialize the master, configure all slaves registered with registerSlave() 
    * 
    * On return, all slaves will be in SAFE_OP mode. Also, the EtherCAT house holding thread
    * will be started. 
    * 
    * If DC is enabled, the ethercat nodes will be switched to OP as soon as synchronization
    * is attained between the slave clocks and the master
    * 
    * If DC is disabled, the slaves will be switched to OP as soon as they come online.
    * 
    * @throws IOException
    */
   public void init() throws IOException
   {
      trace("Setting up fast IRQ");
      setupFastIRQ(iface);

      trace("Creating context");
      context = soem.ecx_create_context();
      
      trace("Context created");
      
      if(soem.ecx_init(context, iface) == 0)
      {
         throw new IOException("Cannot open interface " + iface + ". Make sure to run as root.");
      }
      
      
      trace("Opened interface");
      
      
      if(soem.ecx_config_init(context, (short)0) == 0)
      {
         throw new IOException("Cannot initialize slaves");
      }
      trace("Configured slaves");
      
      if(enableDC)
      {
         boolean dcCapable = soem.ecx_configdc(context) == (short)1;
         enableDC = dcCapable;
         ec_slavet allSlaves = soem.ecx_slave(context, 0);
         int masterClock = allSlaves.getDCnext();
         
         slaves.get(masterClock - 1).setDCMasterClock(true);
         
         trace("DC ENABLED " + enableDC + " master clock is " + masterClock);
      }
      
      int slavecount = soem.ecx_slavecount(context);
      
      trace("Got slave count");
      if(slaves.size() != slavecount)
      {
         if(slaves.size() < slavecount)
         {
            throw new IOException("Not all slaves are configured, got " + slavecount + " slaves, expected " + slaves.size());
         }
         else
         {
            throw new IOException("Not all slaves are online, got " + slavecount + " slaves, expected " + slaves.size());
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
            
            
            trace(slave.toString());
            slave.configure(context, ec_slave, i + 1, enableDC, cycleTimeInNs);
            slaveMap[i] = slave;
            processDataSize += slave.processDataSize();
         }
         else
         {
            throw new IOException("Unconfigured slave on alias " + alias + ":" + position + ". Make sure to power cycle after changing alias addresses.");
         }
         
         // Disable Complete Access reading of SDO configuration.
         short config = ec_slave.getCoEdetails();
         config &= ~soemConstants.ECT_COEDET_SDOCA;
         ec_slave.setCoEdetails(config);
         
         previousAlias = alias;
         previousPosition = position;
         
      }
      trace("Configured slaves");
      

      
      
      ioMap = ByteBuffer.allocateDirect(processDataSize);
      ioMap.order(ByteOrder.nativeOrder());
      
      
      int ioBufferSize = soem.ecx_config_map_group(context, ioMap, (short)0);
      if(ioBufferSize != processDataSize)
      {
         throw new IOException("Cannot allocate memory for etherCAT I/O. Expected process size is " + processDataSize + ", allocated " + ioBufferSize);
      }      

      trace("Configured map");

      if(soem.ecx_statecheck(context, 0, ec_state.EC_STATE_SAFE_OP.swigValue(), soemConstants.EC_TIMEOUTSTATE) == 0)
      {
         throw new IOException("Cannot transfer to SAFE_OP state");
      }      
      
      trace("In SAFEOP");
      
      
      
      for(int i = 0; i < slavecount; i++)
      {
         Slave slave = slaveMap[i];
         slave.linkBuffers(ioMap);
      }
      trace("Linked buffers");
      
      send();
      receive();

      
      etherCATHouseHolder.start();
      
      Runtime.getRuntime().addShutdownHook(new Thread()
      {
         public void run()
         {
            shutdown();
         }
      });
      
      
   }
   
   private void setupFastIRQ(String iface) throws IOException
   {
      int ret = soem.ecx_setup_socket_fast_irq(iface);
      
      switch(ret)
      {
      case 10:
         System.err.println("Cannot setup fast IRQ settings on network card. OS is not Linux");
         break;
      case 70:
         throw new IOException("Cannot open control socket to setup network card");
      case 76:
         throw new IOException("Cannot read current coalesce options from network card");
      case 81:
         throw new IOException("Cannot write desired coalesce options to network card");
      case 1:
         //sucess
         break;
      default:
         throw new IOException("Unknown return value " + ret + " while setting coalesce options");
         
      }
   }

   /**
    * Stop the EtherCAT master
    * 
    */
   public void shutdown()
   {
      trace("Waiting for EtherCAT Controller to stop");
      etherCATController.stopController();
      try
      {
         etherCATController.join();
      }
      catch (InterruptedException e)
      {
      }
      
      trace("Shutting down controller thread");
      etherCATHouseHolder.stopExecution();
      
      
      boolean allSlavesShutdown = false;
      
      while(!allSlavesShutdown)
      {
         allSlavesShutdown = true;
         for(int i = 0; i < slaves.size(); i++)
         {
            if(!slaves.get(i).hasShutdown())
            {
               allSlavesShutdown = false;
               slaves.get(i).shutdown();
            }
         }
         
         LockSupport.parkNanos(cycleTimeInNs);
      }
      
      
      trace("Switching slaves to PRE-OP state");
      ec_slavet allSlaves = soem.ecx_slave(context, 0);
      allSlaves.setState(ec_state.EC_STATE_PRE_OP.swigValue());
      soem.ecx_writestate(context, 0);
      
      trace("Cleanup slaves");
      
      for(int i = 0; i < slaves.size(); i ++)
      {
         slaves.get(i).cleanup();
      }
      
      trace("Switching slaves to INIT state");
      allSlaves.setState(ec_state.EC_STATE_INIT.swigValue());
      soem.ecx_writestate(context, 0);
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
    * Read the process data. 
    * 
    * Call after send()
    */
   public void receive()
   {
      soem.ecx_receive_processdata(context, soemConstants.EC_TIMEOUTRET);
   }

   
   /**
    * Register a slave. 
    * 
    * The slave will be setup for cyclical operation when the master initializes. Call before init().
    * @param slave
    */
   public void registerSlave(Slave slave)
   {
      slaves.add(slave);
   }
   
   
   /**
    * Gets the time of the DC Master clcok from the last datagram.
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
   
   private class EtherCATMasterHouseHolder extends Thread
   {
      private volatile boolean running = false;
      private final ArrayList<SDO> SDOs = new ArrayList<>();
      
      public EtherCATMasterHouseHolder()
      {
         super("EtherCATController");
      }
      
      private void addSDO(SDO sdo)
      {
         if(running)
         {
            // Massive threading problems will be introduced when this is allowed.
            throw new RuntimeException("Cannot register new SDOs after the master has started");
         }
         
         SDOs.add(sdo);
      }
      
      @Override
      public void run()
      {         
         while(running)
         {
            soem.ecx_readstate(context);

            for(int i = 0; i < slaves.size(); i++)
            {
               slaves.get(i).doEtherCATStateControl();
            }
            
            for(int i = 0; i < SDOs.size(); i++)
            {
               SDOs.get(i).updateInMasterThread();
            }
            
            try
            {
               Thread.sleep(10);
            }
            catch (InterruptedException e)
            {
               // Ignore
            }
         }
         
      }
      
      @Override
      public void start()
      {
         running = true;
         super.start();
      }
      
      public void stopExecution()
      {
         running = false;
         interrupt();
         try
         {
            join();
         }
         catch (InterruptedException e)
         {
         }
      }
   }
}
