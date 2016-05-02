package us.ihmc.soem.tools;

import java.io.IOException;

import us.ihmc.soem.generated.ec_slavet;
import us.ihmc.soem.generated.ecx_contextt;
import us.ihmc.soem.generated.soem;

public class PrintEtherCATSlaveInfo
{
   public static void main(String[] args) throws IOException
   {
      String iface = args[0];
      
      ecx_contextt context = soem.ecx_create_context();
      
      if(soem.ecx_init(context, iface) == 0)
      {
         throw new IOException("Cannot open interface " + iface);
      }
      
      
      if(soem.ecx_config_init(context, (short)0) == 0)
      {
         throw new IOException("Cannot initialize slaves");
      }
      
      int slavecount = soem.ecx_slavecount(context);
      
      System.out.println("Found " + slavecount + " slaves");
      
      for(int i = 1; i < slavecount; i++)
      {
         ec_slavet ec_slave = soem.ecx_slave(context, i);
         soem.ecx_readstate(context);
         
         StringBuilder b = new StringBuilder();
         b.append(ec_slave.getAliasadr());
         b.append(":");
         b.append(ec_slave.getConfigadr());
         b.append(" ");
         b.append(ec_slave.getName());
      }
      
      
      
   }
}
