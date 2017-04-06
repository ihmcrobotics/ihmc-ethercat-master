/*
 * Copyright 2017 Florida Institute for Human and Machine Cognition (IHMC)
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *     
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. 
 */
package us.ihmc.etherCAT.master;

/** 
 * Internal holder to make sense of bit/byte alignment 
 * 
 * @author Jesper Smith
 *
 */
class BufferOffsetHolder
{


   private final int bitSize;
   private int byteOffset = 0;
   private int bitOffset = 0;

   public BufferOffsetHolder(int byteOffset, int bitOffset, int bitSize)
   {
      this.bitSize = byteOffset * 8 + bitOffset + bitSize;
      increase(byteOffset, bitOffset);
   }

   public int getByteOffset()
   {
      return byteOffset;
   }

   public int getBitOffset()
   {
      return bitOffset;
   }
   
   /**
    * Align the offsets on a byte boundary if bytes > 0
    * 
    * 
    * @param bytes
    * @param bits
    */
   public void align(int bytes, int bits)
   {
      bytes += bits >> 3;
      if(bytes > 0 && bitOffset > 0)
      {
         byteOffset++;
         bitOffset = 0;
      }
   }
   
   /**
    * Increase the offsets, wrapping bits into bytes.
    * 
    * @param bytes
    * @param bits
    * @return true if space is available, false if there is not enough space for the increase
    */
   public boolean increase(int bytes, int bits)
   {
      int newBitOffset = bitOffset + bits;
      int newByteOffset = byteOffset + bytes + (newBitOffset >> 3);
      newBitOffset &= 7;
      
      if(((newByteOffset << 3) + newBitOffset) > bitSize)
      {
         return false;
      }
      else
      {
         bitOffset = newBitOffset;
         byteOffset = newByteOffset;
         return true;
      }
      
   }

   
   


}
