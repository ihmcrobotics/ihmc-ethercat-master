package us.ihmc.etherCAT.slaves;

import java.io.IOException;

public class EasyCATActuatorLowerJointWithButtons extends EasyCATActuatorLowerJoint
{

   private boolean buttons[] = new boolean[4];

   public EasyCATActuatorLowerJointWithButtons(int alias, int ringPosition) throws IOException
   {
      super(alias, ringPosition);
   }

   @Override
   protected void processDataFromLowerJoint()
   {
      super.processDataFromLowerJoint();
   }

   private void processWirelessButtons()
   {
   }
   
   public boolean[] getButtonStates()
   {
      return buttons;
   }
   
   public boolean getLoadButtonState()
   {
      return buttons[0];
   }

   public boolean getUnloadButtonState()
   {
      return buttons[1];
   }
   
   public boolean getIncrementWeightButtonState()
   {
      return buttons[2];
   }
   
   public boolean getDecrementWeightButtonState()
   {
      return buttons[3];
   }
}
