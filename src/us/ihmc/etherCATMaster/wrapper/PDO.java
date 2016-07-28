package us.ihmc.etherCATMaster.wrapper;

import java.nio.ByteOrder;

import us.ihmc.etherCATMaster.javalution.Struct;

/**
 * PDO class based on Javalution's struct. 
 * 
 * This represents a packed struct, PDO entries have to be defined as they appear in the slaves PDO configuration with the correct size.
 * 
 * @author Jesper Smith
 *
 */
abstract class PDO extends Struct
{
   private final short address;   


   /**
    * 2 bit padding field
    * 
    * @author jesper
    *
    */
   public class Bit2 extends BitField
   {
      public Bit2()
      {
         super(2);
      }
   }
   
   /**
    * 3 bit padding field
    * 
    * @author jesper
    *
    */
   public class Bit3 extends BitField
   {
      public Bit3()
      {
         super(3);
      }
   }

   /**
    * 4 bit padding field
    * 
    * @author jesper
    *
    */
   public class Bit4 extends BitField
   {
      public Bit4()
      {
         super(4);
      }
   }

   
   /**
    * 5 bit padding field
    * 
    * @author jesper
    *
    */
   public class Bit5 extends BitField
   {
      public Bit5()
      {
         super(5);
      }
   }

   
   /**
    * 6 bit padding field
    * 
    * @author jesper
    *
    */
   public class Bit6 extends BitField
   {
      public Bit6()
      {
         super(6);
      }
   }

   
   /**
    * 7 bit padding field
    * 
    * @author jesper
    *
    */
   public class Bit7 extends BitField
   {
      public Bit7()
      {
         super(7);
      }
   }

   PDO(int address)
   {
      this.address = (short) address;
   }

   /**
    * Internal use, do not override
    */
   @Override
   public final ByteOrder byteOrder()
   {
      return ByteOrder.nativeOrder();
   }

   /**
    * Internal use, do not override
    */
   @Override
   public final boolean isPacked()
   {
      return true;
   }

   /**
    * Internal use, do not override
    */
   final short getAddress()
   {
      return address;
   }

}
