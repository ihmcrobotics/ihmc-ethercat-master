package us.ihmc.etherCAT.master;

import us.ihmc.etherCAT.master.Mailbox.MailboxConfiguration;
import us.ihmc.etherCAT.soemJavaNativeLibrary;
import us.ihmc.soem.generated.ec_err_type;
import us.ihmc.soem.generated.ec_slavet;
import us.ihmc.soem.generated.ec_state;
import us.ihmc.soem.generated.ecx_context;
import us.ihmc.soem.generated.soem;
import us.ihmc.soem.generated.soemConstants;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.HashMap;

/**
 * Standalone implementation for File over EtherCAT in BOOT mode
 * 
 * @author Jesper Smith
 *
 */
public class FoEBootstrap
{
   static
   {
      soemJavaNativeLibrary.load();
   }

   private class SlaveConfig
   {
      private int alias;
      private int position;

      @Override
      public int hashCode()
      {
         return alias << 16 + position;
      }

      @Override
      public boolean equals(Object obj)
      {
         if (obj instanceof SlaveConfig)
         {
            return ((SlaveConfig) obj).alias == alias && ((SlaveConfig) obj).position == position;
         }
         else
         {
            return false;
         }
      }

      public String toString()
      {
         return "(" + alias + ", " + position + ")";
      }
   }

   private ecx_context context;

   private final ec_slavet[] slaves;
   private final HashMap<SlaveConfig, Integer> slaveMap = new HashMap<>();
   private final boolean printInfo;

   private void info(String info)
   {
      if (printInfo)
      {
         System.out.println("[FoE] " + info);
      }
   }

   private void info(int alias, int position, ec_slavet slave, String info)
   {
      info("[(" + alias + ", " + position + ") " + slave.getName() + "] " + info);
   }

   public FoEBootstrap(String iface, boolean printInfo) throws IOException
   {
      this.printInfo = printInfo;
      
      context = soem.ecx_create_context(1);

      if (soem.ecx_init(context, iface) == 0)
      {
         throw new IOException("Cannot open interface " + iface + ". Make sure to run as root.");
      }

      if (soem.ecx_config_init(context, (short) 0) == 0)
      {
         throw new IOException("Cannot initialize slaves");
      }

      int slavecount = soem.ecx_slavecount(context);
      if (slavecount == 1)
      {
         info("Found " + slavecount + " slave.");
      }
      else
      {
         info("Found " + slavecount + " slaves.");
      }
      slaves = new ec_slavet[slavecount + 1];

      int previousAlias = 0;
      int previousPosition = -1;
      for (int i = 1; i <= slavecount; i++)
      {
         ec_slavet ec_slave = soem.ecx_slave(context, i);

         SlaveConfig slaveConfig = new SlaveConfig();
         if (ec_slave.getAliasadr() == 0 || ec_slave.getAliasadr() == previousAlias)
         {
            slaveConfig.alias = previousAlias;
            slaveConfig.position = previousPosition + 1;
         }
         else
         {
            slaveConfig.alias = ec_slave.getAliasadr();
            slaveConfig.position = 0;
         }

         previousAlias = slaveConfig.alias;
         previousPosition = slaveConfig.position;

         slaves[i] = ec_slave;
         slaveMap.put(slaveConfig, i);
      }

      int statecheck = soem.ecx_statecheck(context, 0, ec_state.EC_STATE_INIT.swigValue(), soemConstants.EC_TIMEOUTSTATE * 4);
      if (statecheck != ec_state.EC_STATE_INIT.swigValue())
      {
         throw new IOException("Cannot transfer to INIT state");
      }

   }

   private int getSlaveIndex(int alias, int position) throws IOException
   {
      SlaveConfig requestedConfig = new SlaveConfig();
      requestedConfig.alias = alias;
      requestedConfig.position = position;

      Integer slaveIndex = slaveMap.get(requestedConfig);
      if (slaveIndex == null)
      {
         throw new IOException("Cannot open slave with alias " + requestedConfig);
      }

      return slaveIndex;
   }

   private int getSlaveInBootMode(int alias, int position) throws IOException
   {
      int slaveIndex = getSlaveIndex(alias, position);
      ec_slavet ec_slave = slaves[slaveIndex];

      info(alias, position, ec_slave, "Switching to INIT");
      ec_slave.setState(ec_state.EC_STATE_INIT.swigValue());
      soem.ecx_writestate(context, slaveIndex);

      if (soem.ecx_statecheck(context, slaveIndex, ec_state.EC_STATE_INIT.swigValue(), soemConstants.EC_TIMEOUTSTATE) != ec_state.EC_STATE_INIT.swigValue())
      {
         throw new IOException("Cannot switch slave to INIT state");
      }

      info(alias, position, ec_slave, "Configuring BOOT mailbox");
      Mailbox.setup(context, slaveIndex, MailboxConfiguration.BOOT);
      if(printInfo)
      {
         Mailbox.printConfiguration(ec_slave);
      }

      info(alias, position, ec_slave, "Switching to BOOT");
      ec_slave.setState(ec_state.EC_STATE_BOOT.swigValue());
      soem.ecx_writestate(context, slaveIndex);

      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
      }
      if (soem.ecx_statecheck(context, slaveIndex, ec_state.EC_STATE_BOOT.swigValue(), soemConstants.EC_TIMEOUTSTATE) == ec_state.EC_STATE_BOOT.swigValue())
      {
         return slaveIndex;
      }
      else
      {
         throw new IOException("Cannot switch to BOOT state.");
      }
   }

   private void setSlaveToInit(int alias, int position) throws IOException
   {
      int slaveIndex = getSlaveIndex(alias, position);
      ec_slavet ec_slave = slaves[slaveIndex];

      info(alias, position, ec_slave, "Configuring default mailbox");
      Mailbox.setup(context, slaveIndex, MailboxConfiguration.DEFAULT);
      if(printInfo)
      {
         Mailbox.printConfiguration(ec_slave);
      }

      info(alias, position, ec_slave, "Switching to INIT");
      ec_slave.setState(ec_state.EC_STATE_INIT.swigValue());
      soem.ecx_writestate(context, slaveIndex);
      if (soem.ecx_statecheck(context, slaveIndex, ec_state.EC_STATE_INIT.swigValue(),
                              soemConstants.EC_TIMEOUTSTATE * 10) != ec_state.EC_STATE_INIT.swigValue())
      {
         throw new IOException("Cannot restore slave to INIT state");
      }

   }

   public void writeDataToSlave(int alias, int position, String filename, long password, ByteBuffer data) throws IOException
   {
      int slaveIndex = getSlaveInBootMode(alias, position);

      ec_slavet ec_slave = slaves[slaveIndex];
      info(alias, position, ec_slave, "Writing " + data.remaining() + " bytes to " + filename);
      int result = soem.ecx_FOEwrite(context, slaveIndex, filename, password, data.remaining(), data, soemConstants.EC_TIMEOUTSTATE);
      if (result > 0)
      {
         setSlaveToInit(alias, position);
      }
      else
      {
         throw new IOException("Cannot read write data to slave. Error code: " + ec_err_type.swigToEnum(-result));
      }

   }

   public ByteBuffer readDataFromSlave(int alias, int position, String filename, long password, int maxSize) throws IOException
   {
      int slaveIndex = getSlaveInBootMode(alias, position);

      ec_slavet ec_slave = slaves[slaveIndex];
      info(alias, position, ec_slave, "Reading " + filename);

      ByteBuffer data = ByteBuffer.allocateDirect(maxSize);
      data.order(ByteOrder.LITTLE_ENDIAN);
      int result = soem.ecx_FOEread_java_helper(context, slaveIndex, filename, password, maxSize, data, soemConstants.EC_TIMEOUTSTATE);

      if (result > 0)
      {
         data.limit(result);
         info(alias,position, ec_slave, "Read " + data.limit() + " bytes");
         setSlaveToInit(alias, position);
         return data;
      }
      else
      {
         throw new IOException("Cannot read data from slave. Error code: " + ec_err_type.swigToEnum(-result));

      }
   }

   public void close()
   {
      soem.ecx_close(context);
   }

}
