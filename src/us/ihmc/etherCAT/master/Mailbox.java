package us.ihmc.etherCAT.master;

import us.ihmc.soem.generated.ec_slavet;
import us.ihmc.soem.generated.ec_smt;
import us.ihmc.soem.generated.ecx_contextt;
import us.ihmc.soem.generated.soem;
import us.ihmc.soem.generated.soemConstants;

public class Mailbox
{
   public enum MailboxConfiguration   
   {
      BOOT,
      DEFAULT
   }
   
   static int LO_WORD(long l) 
   {
      return (int) ((l) & 0xffffl);
   }
   
   static int HI_WORD(long l) 
   {
      return (int) ((l) >> 16l); 
   }

   static void setup(ecx_contextt context, int slaveIndex, MailboxConfiguration config)
   {
      long data;
      
      int rxMailbox, txMailbox;
      if(config == MailboxConfiguration.BOOT)
      {
         rxMailbox = soemConstants.ECT_SII_BOOTRXMBX;
         txMailbox = soemConstants.ECT_SII_BOOTTXMBX;
      }
      else
      {
         rxMailbox = soemConstants.ECT_SII_RXMBXADR;
         txMailbox = soemConstants.ECT_SII_TXMBXADR;
      }
      
      
      
      ec_slavet ec_slave = soem.ecx_slave(context, slaveIndex);
      
      ec_smt sm0 = soem.ecx_sm(ec_slave, 0);
      ec_smt sm1 = soem.ecx_sm(ec_slave, 1);
      

      // Read master -> slave mailbox data
      data = soem.ecx_readeeprom(context, slaveIndex, rxMailbox, soem.EC_TIMEOUTEEP);
      sm0.setStartAddr(LO_WORD(data));
      sm0.setSMlength(HI_WORD(data));
      
      ec_slave.setMbx_wo(LO_WORD(data));
      ec_slave.setMbx_l(HI_WORD(data));
      
      
      // Read slave -> master mailbox data
      data = soem.ecx_readeeprom(context, slaveIndex, txMailbox, soem.EC_TIMEOUTEEP);
      sm1.setStartAddr(LO_WORD(data));
      sm1.setSMlength(HI_WORD(data));
      
      ec_slave.setMbx_ro(LO_WORD(data));
      ec_slave.setMbx_rl(HI_WORD(data));

      
      
      
      
      
      soem.ecx_writembxconfig(context, ec_slave);

   }
   
   public static void printConfiguration(ec_slavet ec_slave)
   {
      ec_smt sm0 = soem.ecx_sm(ec_slave, 0);
      ec_smt sm1 = soem.ecx_sm(ec_slave, 1);

      System.out.println("\tSM0 : address: 0x" + Integer.toHexString(sm0.getStartAddr()) + " length: " + sm0.getSMlength() + " flags: 0x" + Integer.toHexString((int) sm0.getSMflags()));
      System.out.println("\tSM1 : address: 0x" + Integer.toHexString(sm1.getStartAddr()) + " length: " + sm1.getSMlength() + " flags: 0x" + Integer.toHexString((int) sm1.getSMflags()));
   }
}
