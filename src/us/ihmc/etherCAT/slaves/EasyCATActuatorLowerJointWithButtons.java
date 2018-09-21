package us.ihmc.etherCAT.slaves;

import java.io.IOException;

public class EasyCATActuatorLowerJointWithButtons extends EasyCATActuatorLowerJoint
{

   private boolean buttonStates[] = new boolean[4];

   public EasyCATActuatorLowerJointWithButtons(int alias, int ringPosition) throws IOException
   {
      super(alias, ringPosition);
   }

   @Override
   protected void processDataFromLowerJoint()
   {
      super.processDataFromLowerJoint();
      processWirelessButtons();
   }

   private void processWirelessButtons()
   {
      if (frameData[5] == 1)//Pressed
      {
         buttonStates[0] = true;
      }
      else
      {
         buttonStates[0] = false;
      }

      if (frameData[6] == 1)//Pressed
      {
         buttonStates[1] = true;
      }
      else
      {
         buttonStates[1] = false;
      }

      if (frameData[7] == 1)//Pressed
      {
         buttonStates[2] = true;
      }
      else
      {
         buttonStates[2] = false;
      }

      if (frameData[8] == 1)//Pressed
      {
         buttonStates[3] = true;
      }
      else
      {
         buttonStates[3] = false;
      }
   }

   public boolean[] getButtonStates()
   {
      return buttonStates;
   }

   public boolean getLoadButtonState()
   {
      return buttonStates[0];
   }

   public boolean getUnloadButtonState()
   {
      return buttonStates[1];
   }

   public boolean getIncrementWeightButtonState()
   {
      return buttonStates[2];
   }

   public boolean getDecrementWeightButtonState()
   {
      return buttonStates[3];
   }
}
