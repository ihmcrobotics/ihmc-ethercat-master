package us.ihmc.etherCAT.master;

import us.ihmc.etherCAT.soemJavaNativeLibrary;
import us.ihmc.soem.generated.ec_slavet;
import us.ihmc.soem.generated.ec_smt;
import us.ihmc.soem.generated.ecx_context;
import us.ihmc.soem.generated.soem;
import us.ihmc.soem.generated.soemConstants;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Simple application to print basic information about the detected slave configuration
 * 
 * @author Jesper Smith
 *
 */
public class SlaveInfo
{
   private static final short FALSE = (short) 0;
   private static final short TRUE = (short) 1;

   static
   {
      soemJavaNativeLibrary.load();
   }

   public static String hex(long n)
   {
      return String.format("0x%8s", Long.toHexString(n)).replace(' ', '0');
   }

   public static String hex(int n)
   {
      return String.format("0x%4s", Integer.toHexString(n)).replace(' ', '0');
   }

   public static String hex(short n)
   {
      return String.format("0x%2s", Integer.toHexString(n)).replace(' ', '0');
   }

   /** Read PDO assign structure 
    * @param type */
   private static void printSDOAssignment(ecx_context context, int slaveIndex, int PDOassign, int type)
   {
      ByteBuffer buffer = ByteBuffer.allocateDirect(8);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      ServiceObjectEntryReader reader = new ServiceObjectEntryReader(context);
      ServiceObjectDescriptionReader serviceObjectDescriptionReader = new ServiceObjectDescriptionReader(context);

      /* read PDO assign subindex 0 ( = number of PDO's) */
      int wkc = soem.ecx_SDOread_java_helper(context, slaveIndex, PDOassign, (short) 0x00, FALSE, 2, buffer, soemConstants.EC_TIMEOUTRXM);
      int numberOfPDOS = buffer.getShort(0) & 0xFFFF;
      /* positive result from slave ? */
      if ((wkc > 0) && (numberOfPDOS > 0))
      {
         /* read all PDO's */
         for (int i = 1; i <= numberOfPDOS; i++)
         {
            /* read PDO assign */
            wkc = soem.ecx_SDOread_java_helper(context, slaveIndex, PDOassign, (short) i, FALSE, 2, buffer, soemConstants.EC_TIMEOUTRXM);

            /* result is index of PDO */
            int idx = buffer.getShort(0) & 0xFFFF;
            if (wkc > 0 && idx > 0)
            {
               String pdoType = type == 3 ? "RxPDO" : "TxPDO";
               if (serviceObjectDescriptionReader.readObjectDescription(slaveIndex, idx))
               {
                  System.out.println("\t\t" + pdoType + " " + hex(idx) + "\t" + serviceObjectDescriptionReader.getName());
               }
               else
               {
                  System.out.println("\t\t" + pdoType + " " + hex(idx));
               }

               wkc = soem.ecx_SDOread_java_helper(context, slaveIndex, idx, (short) 0x0, FALSE, 1, buffer, soemConstants.EC_TIMEOUTRXM);
               int count = buffer.get(0) & 0xFF;
               if (wkc > 0 && count > 0)
               {
                  for (int subCount = 1; subCount <= count; subCount++)
                  {
                     wkc = soem.ecx_SDOread_java_helper(context, slaveIndex, idx, (short) subCount, FALSE, 4, buffer, soemConstants.EC_TIMEOUTRXM);
                     int entryIndex = buffer.getShort(2) & 0xFFFF;
                     short entrySubIndex = (short) (buffer.get(1) & 0xFF);
                     int length = buffer.get(0) & 0xFF;
                     if (wkc > 0)
                     {

                        if ((entryIndex != 0 || entrySubIndex != 0) && reader.readSingleOE(slaveIndex, entryIndex, entrySubIndex))
                        {
                           System.out.println("\t\t\t" + hex(entryIndex) + ":" + hex(entrySubIndex) + " " + reader.getDataTypeAsString() + "\t" + reader.getName());
                        }
                        else
                        {
                           System.out.println("\t\t\t" + hex(entryIndex) + ":" + hex(entrySubIndex) + " (" + length + " bit)");
                        }

                     }

                  }
               }

            }
         }
      }
   }

   public static void showOptions()
   {
      System.out.println("Usage:  SlaveInfo [network interface] [options]");
      System.out.println("[options] can be");
      System.out.println("\t--pdo\tShow PDO configuration");
      System.out.println("\t-h\tShow this message");
      
   }
   
   public static void main(String[] args) throws IOException
   {

      if (args.length == 0)
      {
         showOptions();
         System.exit(-1);
      }
      String iface = args[0];
      
      
      boolean printPDO = false;
      for(int i = 1; i < args.length; i++)
      {
         if(args[i].equals("--pdo"))
         {
            printPDO = true;
         }
         else if (args[i].equals("-h"))
         {
            showOptions();
            System.exit(0);
         }
      }

      ecx_context context = soem.ecx_create_context(1);

      if (soem.ecx_init(context, iface) == 0)
      {
         throw new IOException("Cannot open interface " + iface + ". Make sure to run as root.");
      }

      if (soem.ecx_config_init(context, (short) 0) == 0)
      {
         throw new IOException("Cannot initialize slaves");
      }

      int slavecount = soem.ecx_slavecount(context);

      System.out.println("Found " + slavecount + " slaves");
      int previousAlias = 0;
      int previousPosition = -1;
      for (int slaveIndex = 1; slaveIndex <= slavecount; slaveIndex++)
      {
         ec_slavet ec_slave = soem.ecx_slave(context, slaveIndex);

         int alias, position;
         if (ec_slave.getAliasadr() == 0 || ec_slave.getAliasadr() == previousAlias)
         {
            alias = previousAlias;
            position = previousPosition + 1;
         }
         else
         {
            alias = ec_slave.getAliasadr();
            position = 0;
         }

         System.out.println(slaveIndex + " - " + alias + ":" + position + " " + ec_slave.getName());

         System.out.println("\tManufacturer: " + hex(ec_slave.getEep_man()));
         System.out.println("\tProduct code: " + hex(ec_slave.getEep_id()));
         System.out.println("\tRevision: " + ec_slave.getEep_rev());
         System.out.println("\tDistributed Clocks: " + (ec_slave.getHasdc() == TRUE ? "yes" : "no"));

         for (int nSM = 0; nSM < soemConstants.EC_MAXSM; nSM++)
         {
            ec_smt sm = soem.ecx_sm(ec_slave, nSM);
            if (sm.getStartAddr() > 0)
            {
               String type;
               ;
               switch (soem.ecx_smtype(ec_slave, nSM))
               {
               case 1:
                  type = "Mailbox messages receive";
                  break;
               case 2:
                  type = "Mailbox messages transmit";
                  break;
               case 3:
                  type = "Cyclic process data receive";
                  break;
               case 4:
                  type = "Cyclic process data transmit";
                  break;
               default:
                  type = "Unused";
                  break;
               }

               System.out.println("\tSM(" + nSM + ") Address: " + hex(sm.getStartAddr()) + ", length: " + sm.getSMlength() + "\tFlags: " + sm.getSMflags() + "\tType: " + type);

               if (printPDO && (ec_slave.getMbx_proto() & soem.ECT_MBXPROT_COE) > 0)
               {
                  if (soem.ecx_smtype(ec_slave, nSM) == 3 || soem.ecx_smtype(ec_slave, nSM) == 4)
                  {
                     printSDOAssignment(context, slaveIndex, soem.ECT_SDO_PDOASSIGN + nSM, soem.ecx_smtype(ec_slave, nSM));
                  }
               }
            }

         }

         previousAlias = alias;
         previousPosition = position;

      }
   }
}
