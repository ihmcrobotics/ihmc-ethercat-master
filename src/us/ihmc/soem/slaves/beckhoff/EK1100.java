package us.ihmc.soem.slaves.beckhoff;

import us.ihmc.soem.wrapper.Slave;

public class EK1100 extends Slave
{
   
   static final int vendorID = 0x00000002;
   static final int productCode = 0x044c2c52;

   public EK1100(int aliasAddress, int configAddress)
   {
      super(vendorID, productCode, aliasAddress, configAddress);
   }


}
