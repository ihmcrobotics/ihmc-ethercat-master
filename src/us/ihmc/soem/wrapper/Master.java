package us.ihmc.soem.wrapper;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

import us.ihmc.soem.generated.ec_slavet;
import us.ihmc.soem.generated.ec_state;
import us.ihmc.soem.generated.ecx_contextt;
import us.ihmc.soem.generated.soem;
import us.ihmc.soem.generated.soemConstants;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class Master
{
   static
   {
      NativeLibraryLoader.loadLibrary("us.ihmc.soem.generated", "soemJava");
   }
   
   
   private final ArrayList<Slave> slaves = new ArrayList<>();
   private final ArrayList<SlaveShutdownHook> shutdownHooks = new ArrayList<>();
   private final String iface;
   
   private ecx_contextt context = null;
   private Slave[] slaveMap;
   
   private ByteBuffer ioMap;
   
   private boolean enableDC = false;
   private long dcCycleTime = 0;
   private long dcControlIntegral = 0;
   
   /**
    * Create new EtherCAT master.
    * 
    * @param iface The network interface to listen on
    */
   public Master(String iface)
   {
      this.iface = iface;
   }
      
   @Deprecated
   public void activate() throws IOException
   {
      init();
   }
   
   private Slave getSlave(ec_slavet ec_slave)
   {
      for(int i = 0; i < slaves.size(); i++)
      {
         Slave slave = slaves.get(i);
         
         if(slave.getAliasAddress() == ec_slave.getAliasadr() &&
               slave.getConfigAddress() == ec_slave.getConfigindex())
         {
            return slave;
         }
      }
      return null;
   }
   
   public void configureDC(long cycleTime)
   {
      enableDC = true;
      dcCycleTime = cycleTime;
   }
   
   /**
    * Initialize the master, configure all slaves registered with registerSlave() 
    * 
    * On return, all slaves will be in OPERATIONAL mode.
    * 
    * @throws IOException
    */
   public void init() throws IOException
   {
      context = soem.ecx_create_context();
      
      if(soem.ecx_init(context, iface) == 0)
      {
         throw new IOException("Cannot open interface " + iface);
      }
      
      
      if(soem.ecx_config_init(context, (short)0) == 0)
      {
         throw new IOException("Cannot initialize slaves");
      }
      
      int slavecount = soem.ecx_slavecount(context);
      
      if(slaves.size() != slavecount)
      {
         throw new IOException("Not all slaves are online, got " + slavecount + " slaves, expected " + slaves.size());
      }
      
      slaveMap = new Slave[slavecount];
      int processDataSize = 0;
      for(int i = 0; i < slavecount; i++)
      {
         ec_slavet ec_slave = soem.ecx_slave(context, i + 1);
         Slave slave = getSlave(ec_slave);
         
         if(slave != null)
         {
            slave.configure(context, ec_slave, i + 1);
            slaveMap[i] = slave;
            processDataSize += slave.processDataSize();
         }
         else
         {
            throw new IOException("Unconfigured slave on alias " + ec_slave.getAliasadr());
         }
         
         // Disable Complete Access reading of SDO configuration.
         short config = ec_slave.getCoEdetails();
         config &= ~soemConstants.ECT_COEDET_SDOCA;
         ec_slave.setCoEdetails(config);
         
         
      }
      ioMap = ByteBuffer.allocateDirect(processDataSize);
      ioMap.order(ByteOrder.nativeOrder());
      
      
      int ioBufferSize = soem.ecx_config_map_group(context, ioMap, (short)0);
      if(ioBufferSize != processDataSize)
      {
         throw new IOException("Cannot allocate memory for etherCAT I/O. Expected process size is " + processDataSize + ", allocated " + ioBufferSize);
      }
      
      if(soem.ecx_statecheck(context, 0, ec_state.EC_STATE_SAFE_OP.swigValue(), soemConstants.EC_TIMEOUTSTATE) == 0)
      {
         throw new IOException("Cannot transfer to SAFE_OP state");
      }      
      
      for(int i = 0; i < slavecount; i++)
      {
         Slave slave = slaveMap[i];
         slave.linkBuffers(ioMap);
      }
      
      
      if(enableDC)
      {
         boolean dcCapable = soem.ecx_configdc(context) == (short)1;
         enableDC = dcCapable;
      }
      
      soem.ecx_send_processdata(context);
      soem.ecx_receive_processdata(context, soemConstants.EC_TIMEOUTRET);

      
      ec_slavet allSlaves = soem.ecx_slave(context, 0);
      allSlaves.setState(ec_state.EC_STATE_OPERATIONAL.swigValue());
      soem.ecx_writestate(context, 0);
      
      int chk = 40;
      /* wait for all slaves to reach OP state */
      do
      {
         soem.ecx_send_processdata(context);
         soem.ecx_receive_processdata(context, soemConstants.EC_TIMEOUTRET);
         soem.ecx_statecheck(context, 0, ec_state.EC_STATE_OPERATIONAL.swigValue(), 50000);
      }
      while ((chk-- > 0) && (allSlaves.getState() != ec_state.EC_STATE_OPERATIONAL.swigValue()));
      

      /* wait for all slaves to reach OP state */
      if (allSlaves.getState() != ec_state.EC_STATE_OPERATIONAL.swigValue())
      {
         
         soem.ecx_readstate(context);

         for(Slave slave : slaveMap)
         {
            System.out.println(slave.toString() + " [" + slave.getState() + "], AL Status: " + slave.getALStatusMessage());
         }
         
         throw new IOException("Cannot bring all slaves in OP state");
      }
   }
   
   public void shutdown()
   {
      
   }
   
   public void send()
   {
      soem.ecx_send_processdata(context);
   }
   
   public void receive()
   {
      soem.ecx_receive_processdata(context, soemConstants.EC_TIMEOUTRET);
   }

   public void registerSlave(Slave slave)
   {
      slaves.add(slave);
   }

   public void addSlaveShutDownHook(SlaveShutdownHook shutdownHook)
   {
   }
   
   
   /* PI calculation to get linux time synced to DC time */
   
   /**
    * Simple PI controller to be used to synchronize the control loop with the 
    * Distributed Clocks feature of EtherCAT.   
    * 
    * @param clockTime 
    * @param syncOffset Offset from the start of the DC sync pulse.
    * 
    * @return Offset in NS to add to the current tick duration to synchronize the clocks
    */
   public long calculateDCOffsetTime(long clockTime, long syncOffset)
   {
      if(enableDC)
      {
         long reftime = soem.ecx_dcTime(context);
         
         /* set linux sync point 50us later than DC sync, just as example */
         long delta = (reftime - syncOffset) % dcCycleTime;
         if(delta> (dcCycleTime /2)) { delta= delta - dcCycleTime; }
         if(delta>0){ dcControlIntegral++; }
         if(delta<0){ dcControlIntegral--; }
         return -(delta / 100) - (dcControlIntegral /20);
      }
      else
      {
         return 0;
      }
   }

   
   
}
