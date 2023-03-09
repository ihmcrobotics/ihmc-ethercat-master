package us.ihmc.etherCAT.slaves;

import us.ihmc.etherCAT.slaves.easyCAT.EasyCATSlave32;

import java.io.IOException;

public class EasyCATWirelessButtonsSlave extends EasyCATSlave32
{
   private int[] frameData = new int[32];
   private boolean[] buttonStates = new boolean[4];
   private int dataFlipBit = 0;

   public EasyCATWirelessButtonsSlave(int alias, int ringPosition) throws IOException
   {
      super(alias, ringPosition);
   }

   public void processData()
   {

      getTransmitBytes(frameData, 0, 31);
      

      if (frameData[1] == 1)//Pressed
      {
         buttonStates[0] = true;
      }
      else
      {
         buttonStates[0] = false;
      }

      if (frameData[2] == 1)//Pressed
      {
         buttonStates[1] = true;
      }
      else
      {
         buttonStates[1] = false;
      }

      if (frameData[3] == 1)//Pressed
      {
         buttonStates[2] = true;
      }
      else
      {
         buttonStates[2] = false;
      }

      if (frameData[4] == 1)//Pressed
      {
         buttonStates[3] = true;
      }
      else
      {
         buttonStates[3] = false;
      }
      
      dataFlipBit = frameData[5];
   }

   public boolean isButton1Pressed()
   {
      return buttonStates[0];
   }

   public boolean isButton2Pressed()
   {
      return buttonStates[1];
   }

   public boolean isButton3Pressed()
   {
      return buttonStates[2];
   }

   public boolean isButton4Pressed()
   {
      return buttonStates[3];
   }

   public boolean[] getButtonStates()
   {
      return buttonStates;
   }
   
   public int getDataFlipBit()
   {
      return dataFlipBit;
   }
}
