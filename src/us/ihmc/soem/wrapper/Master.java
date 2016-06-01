package us.ihmc.soem.wrapper;

import java.io.IOException;
import java.nio.ByteBuffer;
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
   private final String iface;
   
   private ecx_contextt context = null;
   private Slave[] slaveMap;
   
   private ByteBuffer ioMap;
   
   public Master(String iface)
   {
      this.iface = iface;
   }
   
   
   void addSlave(Slave slave)
   {
      
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
               slave.getConfigAddress() == ec_slave.getConfigadr())
         {
            return slave;
         }
      }
      return null;
   }
   
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
         throw new IOException("Not all slaves are online, got " + slaves.size() + " slaves, expected " + slavecount);
      }
      
      slaveMap = new Slave[slavecount];
      int processDataSize = 0;
      for(int i = 1; i < slavecount; i++)
      {
         ec_slavet ec_slave = soem.ecx_slave(context, i);
         Slave slave = getSlave(ec_slave);
         
         if(slave != null)
         {
            slave.configure(context, ec_slave, i);
            slaveMap[i] = slave;
            processDataSize += slave.processDataSize();
         }
         else
         {
            throw new IOException("Unconfigured slave on alias " + ec_slave.getAliasadr());
         }
         
      }
      ioMap = ByteBuffer.allocateDirect(processDataSize);
      if(soem.ecx_config_map_group(context, ioMap, (short)0) == 0)
      {
         throw new IOException("Cannot allocate memory for etherCAT I/O");
      }
      
      if(soem.ecx_statecheck(context, 0, ec_state.EC_STATE_SAFE_OP.swigValue(), soemConstants.EC_TIMEOUTSTATE) == 0)
      {
         throw new IOException("Cannot transfer to SAFE_OP state");
      }      
      
      for(int i = 1; i < slavecount; i++)
      {
         Slave slave = slaveMap[i];
         slave.linkBuffers(ioMap);
      }
      
      soem.ecx_readstate(context);
      
      
      soem.ecx_send_processdata(context);
      soem.ec_receive_processdata(soemConstants.EC_TIMEOUTRET);

      ec_slavet allSlaves = soem.ecx_slave(context, 0);
      allSlaves.setState(ec_state.EC_STATE_OPERATIONAL.swigValue());
      soem.ecx_writestate(context, 0);

      /* wait for all slaves to reach OP state */
      soem.ecx_statecheck(context, 0, ec_state.EC_STATE_OPERATIONAL.swigValue(),  soemConstants.EC_TIMEOUTSTATE);
      if (allSlaves.getState() != ec_state.EC_STATE_OPERATIONAL.swigValue())
      {
         throw new IOException("Cannot bring all slaves in OP state");
      }
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

   public void addSlaveShutDownHook(SlaveShutdownHook dsp402ShutDownHook)
   {
   }
   
   
}
