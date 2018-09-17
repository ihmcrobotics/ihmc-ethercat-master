package us.ihmc.etherCAT.slaves;

import us.ihmc.etherCAT.slaves.EasyCATSlave;

import java.io.IOException;

public class EasyCATVerticalPositionerSlave extends EasyCATSlave
{

   private int[] frameData = new int[32];
   private int currentPositionInMeters = 0;
   private boolean lowerLimitSwitch = false;
   private boolean upperLimitSwitch = false;
   private boolean manualUpButton = false;
   private boolean manualDownButton = false;
   private double desiredPositionInMeters = 0;

   public EasyCATVerticalPositionerSlave(int alias, int ringPosition) throws IOException
   {
      super(alias, ringPosition);
   }

   public void processDataFromVerticalPositioner()
   {

      getTransmitBytes(frameData, 0, 31);

      currentPositionInMeters = frameData[5];

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
   
   public void processOutputCommands()
   {
      frameData[1] = (int)(desiredPositionInMeters * 100.0);
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
   
   public boolean isManualDownButtonPressed()
   {
      return manualDownButton;
   }
   
   public boolean isManualUpButtonPressed()
   {
      return manualUpButton;
   }
}
