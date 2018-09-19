package us.ihmc.etherCAT.slaves;

import java.io.IOException;

public class EasyCATVerticalPositionerSlaveWithButtons extends EasyCATVerticalPositionerSlave
{

   private boolean manualUpButton = false;
   private boolean manualDownButton = false;

   //TODO : Left and right vertical positioner slaves need to be flashed with correct aliases!
   public EasyCATVerticalPositionerSlaveWithButtons(int alias, int ringPosition) throws IOException
   {
      super(alias, ringPosition);
   }

   @Override
   public void processDataFromVerticalPositioner()
   {

      getTransmitBytes(frameData, 0, 31);

      processPositionEncoder();
      processLimitSwitches();
      processManualButtons();

      dataFlipBit = frameData[10];
   }

   private void processManualButtons()
   {
      if (frameData[8] == 0)
      {
         manualDownButton = false;
      }
      else
      {
         manualDownButton = true;
      }

      if (frameData[9] == 0)
      {
         manualUpButton = false;
      }
      else
      {
         manualUpButton = true;
      }
   }


   public boolean isManualDownButtonPressed()
   {
      return manualDownButton;
   }

   public boolean isManualUpButtonPressed()
   {
      return manualUpButton;
   }
}
