package us.ihmc.etherCAT.slaves;

import us.ihmc.etherCAT.slaves.easyCAT.EasyCATSlave32;

import java.io.IOException;

public class EasyCATVerticalPositionerSlave extends EasyCATSlave32
{

   protected int[] frameData = new int[32];
   protected double currentPositionInMeters = 0;
   protected boolean lowerLimitSwitch = false;
   protected boolean upperLimitSwitch = false;
   protected double desiredPositionInMeters = 0;
   protected int dataFlipBit = 0;

   //TODO : Left and right vertical positioner slaves need to be flashed with correct aliases!
   public EasyCATVerticalPositionerSlave(int alias, int ringPosition) throws IOException
   {
      super(alias, ringPosition);
   }

   public void processDataFromVerticalPositioner()
   {

      getTransmitBytes(frameData, 0, 31);

      processPositionEncoder();
      processLimitSwitches();

      dataFlipBit = frameData[10];
   }

   protected void processLimitSwitches()
   {
      if (frameData[6] == 0)
      {
         lowerLimitSwitch = false;
      }
      else
      {
         lowerLimitSwitch = true;
      }

      if (frameData[7] == 0)
      {
         upperLimitSwitch = false;
      }
      else
      {
         upperLimitSwitch = true;
      }
   }

   protected void processPositionEncoder()
   {
      currentPositionInMeters = (frameData[5] / 100.0);
   }

   public void processOutputCommands()
   {
      frameData[1] = (int) (desiredPositionInMeters * 100.0);
      setReceiveBytes(frameData);
   }

   public void setDesiredPositionInMeters(double desiredPositionInMeters)
   {
      this.desiredPositionInMeters = desiredPositionInMeters;
   }

   public double getCurrentPositionInMeters()
   {
      return currentPositionInMeters;
   }

   public boolean isLowerLimitSwitchPressed()
   {
      return lowerLimitSwitch;
   }

   public boolean isUpperLimitSwitchPressed()
   {
      return upperLimitSwitch;
   }

   public int getDataFlipBit()
   {
      return dataFlipBit;
   }
}
