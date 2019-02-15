package us.ihmc.etherCAT.master;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import us.ihmc.etherCAT.javalution.Struct;

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
    * Internal use, do not override
    */
   final short getAddress()
   {
      return address;
   }

   /**
    * Internal function to link the main buffers to this PDO.
    * 
    * This function deals with the alignment and supports bit-orientated slaves
    *   
    * @param ioMap
    * @param inputOffset
    * @throws IOException
    */
   void linkBuffer(ByteBuffer ioMap, BufferOffsetHolder inputOffset) throws IOException
   {
      int bytesUsed = _length - _wordSize + (_bitsUsed >> 3);
      int bitsUsed = _bitsUsed & 7;
      
      inputOffset.align(bytesUsed, bitsUsed);
      
      setByteBuffer(ioMap, inputOffset.getByteOffset(), inputOffset.getBitOffset());
      
      if(!inputOffset.increase(bytesUsed, bitsUsed))
      {
         throw new IOException("Not enough space available to allocate PDO " + address);
      }
   }

}
