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

public class Master
{
   static
   {
      NativeLibraryLoader.loadLibrary("us.ihmc.soem.generated", "soemJava");
   }
   
   private final EtherCATMasterController etherCATController = new EtherCATMasterController();
   
   private final ArrayList<Slave> slaves = new ArrayList<>();
   private final ArrayList<SlaveShutdownHook> shutdownHooks = new ArrayList<>();
   private final String iface;
   
   private ecx_contextt context = null;
   private Slave[] slaveMap;
   
   private ByteBuffer ioMap;
   
   private boolean enableDC = false;
   
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
         System.out.println(slave);
         System.out.println(ec_slave.getAliasadr());
         if(slave.getAliasAddress() == ec_slave.getAliasadr() &&
               slave.getConfigAddress() == ec_slave.getConfigindex())
         {
            return slave;
         }
      }
      return null;
   }
   
   public void registerReadSDO(ReadSDO sdo)
   {
      etherCATController.addSDO(sdo);
   }
   
   public void enableDC()
   {
      enableDC = true;
   }
   
   /**
    * Initialize the master, configure all slaves registered with registerSlave() 
    * 
    * On return, all slaves will be in SAFE_OP mode.
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
      
      if(enableDC)
      {
         boolean dcCapable = soem.ecx_configdc(context) == (short)1;
         enableDC = dcCapable;
         ec_slavet allSlaves = soem.ecx_slave(context, 0);
         int masterClock = allSlaves.getDCnext();
         
         slaves.get(masterClock - 1).setDCMasterClock(true);
         
         System.out.println("DC ENABLED " + enableDC + " master clock is " + masterClock);
      }
      
      for(int i = 0; i < slavecount; i++)
      {
         Slave slave = slaveMap[i];
         slave.linkBuffers(ioMap);
      }
      
      
      soem.ecx_send_processdata(context);
      soem.ecx_receive_processdata(context, soemConstants.EC_TIMEOUTRET);
//
//      
//       ec_slavet allSlaves = soem.ecx_slave(context, 0);
//       allSlaves.setState(ec_state.EC_STATE_OPERATIONAL.swigValue());
//       soem.ecx_writestate(context, 0);
//       
//       int chk = 40;
//       /* wait for all slaves to reach OP state */
//       do
//       {
//          soem.ecx_send_processdata(context);
//          soem.ecx_receive_processdata(context, soemConstants.EC_TIMEOUTRET);
//          soem.ecx_statecheck(context, 0, ec_state.EC_STATE_OPERATIONAL.swigValue(), 50000);
//       }
//       while ((chk-- > 0) && (allSlaves.getState() != ec_state.EC_STATE_OPERATIONAL.swigValue()));
//       if (allSlaves.getState() != ec_state.EC_STATE_OPERATIONAL.swigValue())
//       {
//          
//          soem.ecx_readstate(context);
//   
//          for(Slave slave : slaveMap)
//          {
//             System.out.println(slave.toString() + " [" + slave.getState() + "], AL Status: " + slave.getALStatusMessage());
//          }
//          
//          throw new IOException("Cannot bring all slaves in OP state");
//       }
      etherCATController.start();
      
      send();
   }
   
   /**
    * This function switches the slave to OP mode. 
    * 
    * This function will return immediatly. The state chang
    * 
    */
   public void switchToOperational()
   {
      
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
   
   
   public long getDCTime()
   {
      return soem.ecx_dcTime(context);
   }
   
   private class EtherCATMasterController extends Thread
   {
      private volatile boolean running = false;
      private final ArrayList<SDO> SDOs = new ArrayList<>();
      
      public EtherCATMasterController()
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
         System.out.println("Starting EtherCAT control thread");
         
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
            
            LockSupport.parkNanos(100000000); // Wait 100ms
         }
         
      }
      
      @Override
      public void start()
      {
         running = true;
         super.start();
      }
   }
}
