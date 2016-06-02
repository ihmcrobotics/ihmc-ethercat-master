package us.ihmc.soem.wrapper;

import java.nio.ByteOrder;

import javolution.io.Struct;

abstract class PDO extends Struct
{
   private final short address;
   
   PDO(int address)
   {
      this.address = (short) address;
   }
   
   @Override
   public final ByteOrder byteOrder()
   {
      return ByteOrder.LITTLE_ENDIAN;
   }
   
   @Override
   public final boolean isPacked()
   {
      return true;
   }

   public short getAddress()
   {
      return address;
   }
   
}
