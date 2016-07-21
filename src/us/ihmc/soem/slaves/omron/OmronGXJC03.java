package us.ihmc.soem.slaves.omron;

import us.ihmc.soem.wrapper.Slave;

public class OmronGXJC03 extends Slave
{
   
   static final int vendorID = 0x00000083;
   static final int productCode = 0x0000007c;

   public OmronGXJC03(int aliasAddress, int configAddress)
   {
      super(vendorID, productCode, aliasAddress, configAddress);
   }


   @Override
   protected void configure(boolean dcEnabled, long cycleTimeInNs)
   {
      if(dcEnabled)
      {
         configureDCSync0(true, cycleTimeInNs, 0);
      }
      else
      {
         configureDCSync0(false, 0, 0);
      }
      
   }
   
   @Override
   protected void shutdown()
   {
    
   }

   @Override
   protected boolean hasShutdown()
   {
      return true;
   }

}
