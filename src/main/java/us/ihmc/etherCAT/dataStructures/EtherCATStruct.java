package us.ihmc.etherCAT.dataStructures;

import java.nio.ByteOrder;

import us.ihmc.etherCAT.javalution.Struct;

/**
 * 
 * This represents a packed struct with the correct byte order for EtherCAT
 * 
 * @author Jesper Smith
 *
 */
public class EtherCATStruct extends Struct
{

   /**
    * Internal use, do not override
    */
   @Override
   public final ByteOrder byteOrder()
   {
      return ByteOrder.LITTLE_ENDIAN;
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
}
