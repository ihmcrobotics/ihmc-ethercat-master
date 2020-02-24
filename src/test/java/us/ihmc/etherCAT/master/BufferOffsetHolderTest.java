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

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class BufferOffsetHolderTest
{
   @Test
   public void test()
   {
      BufferOffsetHolder holder = new BufferOffsetHolder(0, 0, 128);
      
      assertTrue(holder.increase(0, 4));      
      assertEquals(0, holder.getByteOffset());
      assertEquals(4, holder.getBitOffset());
      assertEquals(124, holder.getAvailableBits());

      assertTrue(holder.increase(0, 4));      
      assertEquals(1, holder.getByteOffset());
      assertEquals(0, holder.getBitOffset());
      assertEquals(120, holder.getAvailableBits());
      
      assertTrue(holder.increase(0, 4));      
      assertEquals(1, holder.getByteOffset());
      assertEquals(4, holder.getBitOffset());
      assertEquals(116, holder.getAvailableBits());
      
      assertTrue(holder.increase(1, 6));      
      assertEquals(3, holder.getByteOffset());
      assertEquals(2, holder.getBitOffset());
      assertEquals(102, holder.getAvailableBits());

      assertTrue(holder.increase(0, 6));      
      assertEquals(4, holder.getByteOffset());
      assertEquals(0, holder.getBitOffset());
      assertEquals(96, holder.getAvailableBits());

      assertTrue(holder.increase(3, 7));      
      assertEquals(7, holder.getByteOffset());
      assertEquals(7, holder.getBitOffset());
      assertEquals(65, holder.getAvailableBits());
      
      assertFalse(holder.increase(8, 2));     
      
      BufferOffsetHolder holder2 = new BufferOffsetHolder(12, 5, 128);
      assertTrue(holder2.increase(15, 3));
      assertEquals(5, holder2.getAvailableBits());
      assertTrue(holder2.increase(0, 1));
      assertEquals(4, holder2.getAvailableBits());
      assertTrue(holder2.increase(0, 4));
      assertEquals(0, holder2.getAvailableBits());
      assertFalse(holder2.increase(0, 1));
      
   }
   
   @Test
   public void testAlign()
   {
      BufferOffsetHolder holder = new BufferOffsetHolder(0, 0, 128);
      
      holder.align(0, 3);
      assertTrue(holder.increase(0, 3));
      holder.align(1, 0);
      assertTrue(holder.increase(1, 0));
      
      assertEquals(2, holder.getByteOffset());
      assertEquals(0, holder.getBitOffset());
   }
}
