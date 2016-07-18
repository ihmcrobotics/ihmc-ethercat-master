package us.ihmc.soem.wrapper;

import java.io.IOException;

import us.ihmc.soem.generated.ec_slavet;
import us.ihmc.soem.generated.ecx_contextt;
import us.ihmc.soem.generated.soem;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;


/**
 * Simple application to print basic information about the detected slave configuration
 * 
 * @author Jesper Smith
 *
 */
public class SlaveInfo
{
   static
   {
      NativeLibraryLoader.loadLibrary("us.ihmc.soem.generated", "soemJava");
   }
   
   public static String hex(long n) {
      // call toUpperCase() if that's required
      return String.format("0x%8s", Long.toHexString(n)).replace(' ', '0');
  }
   
   public static void main(String[] args) throws IOException
   {
      
      if(args.length == 0)
      {
         System.out.println("Usage:  SlaveInfo [network interface]");
         System.exit(-1);
      }
      String iface = args[0];
      
      ecx_contextt context = soem.ecx_create_context();
      
      
      if(soem.ecx_init(context, iface) == 0)
      {
         throw new IOException("Cannot open interface " + iface + ". Make sure to run as root.");
      }
      
      
      
      if(soem.ecx_config_init(context, (short)0) == 0)
      {
         throw new IOException("Cannot initialize slaves");
      }
      
      int slavecount = soem.ecx_slavecount(context);
      
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
       
         System.out.println("Slave: " + i);
         System.out.println("    Alias: " + alias);
         System.out.println("    Position: " + position);
         System.out.println("    Manufacturer: " + hex(ec_slave.getEep_man()));
         System.out.println("    Product code: " + hex(ec_slave.getEep_id()));
         System.out.println("    Revision: " + ec_slave.getEep_rev());
         System.out.println("    DC Capable: " + ec_slave.getHasdc());
                  
         System.out.println("    AL Status code: " + soem.ec_ALstatuscode2string(ec_slave.getALstatuscode()));
          
         
         previousAlias = alias;
         previousPosition = position;
         
      }
   }
}
