package us.ihmc.etherCAT.master;

import java.io.IOException;
import java.nio.ByteBuffer;

class UnconfiguredSlave extends Slave
{

   private final String name;
   
   public UnconfiguredSlave(String name, int vendor, int productCode, int aliasAddress, int position)
   {
      super(vendor, productCode, aliasAddress, position);
      this.name = name;
   }
   
   
   @Override
   public String getName()
   {
      return name;
   }
   
   @Override
   void linkBuffers(ByteBuffer ioMap) throws IOException
   {
      // Do nothing here
   }
}
