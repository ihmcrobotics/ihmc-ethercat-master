package us.ihmc.etherCAT.master;

import java.io.IOException;
import java.nio.ByteBuffer;

import us.ihmc.etherCAT.dataStructures.EtherCATStruct;

/**
 * PDO class based on Javalution's struct. 
 * 
 * PDO entries have to be defined as they appear in the slaves PDO configuration with the correct size.
 * 
 * @author Jesper Smith
 *
 */
abstract class PDO extends EtherCATStruct
{
   private final short address;   

   PDO(int address)
   {
      this.address = (short) address;
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
