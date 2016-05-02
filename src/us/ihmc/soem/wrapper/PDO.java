package us.ihmc.soem.wrapper;

import javolution.io.Struct;

abstract class PDO extends Struct
{
   private final short address;
   
   PDO(int address)
   {
      this.address = (short) address;
      
   }
   
   public final boolean isPacked()
   {
      return false;
   }

   public short getAddress()
   {
      return address;
   }
   
}
