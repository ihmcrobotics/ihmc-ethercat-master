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

import static org.junit.Assert.*;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.junit.Test;

public class PDOTest
{

   private static final class BitPDO extends PDO
   {
      Bool a = new Bool();
      Bool b = new Bool();
      Bool c = new Bool();
      Bool d = new Bool();
      
      private BitPDO(int address)
      {
         super(address);
      }
   }
   
   private static final class IntPDO extends PDO
   {
      private IntPDO(int address)
      {
         super(address);
         // TODO Auto-generated constructor stub
      }

      Signed32 uint = new Signed32();
      
   }

   @Test
   public void linkBufferTest() throws IOException
   {
      ByteBuffer ioMap = ByteBuffer.allocate(16 + 12);
      ioMap.order(ByteOrder.LITTLE_ENDIAN);
      BufferOffsetHolder inputOffset = new BufferOffsetHolder(12, 0, 128);
      
      BitPDO a = new BitPDO(0x0000);
      a.linkBuffer(ioMap, inputOffset);
      BitPDO b = new BitPDO(0x0000);
      b.linkBuffer(ioMap, inputOffset);
      BitPDO c = new BitPDO(0x0000);
      c.linkBuffer(ioMap, inputOffset);
      IntPDO d = new IntPDO(0x0000);
      d.linkBuffer(ioMap, inputOffset);
      BitPDO e = new BitPDO(0x0000);
      e.linkBuffer(ioMap, inputOffset);
      
      ioMap.position(12);
      ioMap.put((byte) 0xCA); // 0b11001010
      
      assertFalse(a.a.get());
      assertTrue(a.b.get());
      assertFalse(a.c.get());
      assertTrue(a.d.get());
    
      assertFalse(b.a.get());
      assertFalse(b.b.get());
      assertTrue(b.c.get());
      assertTrue(b.d.get());
      
      ioMap.put((byte) 0xF);
      assertTrue(c.a.get());
      assertTrue(c.b.get());
      assertTrue(c.c.get());
      assertTrue(c.d.get());
      
      ioMap.putInt(532);
      
      assertEquals(532, d.uint.get());
      
      ioMap.put((byte) 0xA);
      assertFalse(e.a.get());
      assertTrue(e.b.get());
      assertFalse(e.c.get());
      assertTrue(e.d.get());
    
      
   }
   
}
