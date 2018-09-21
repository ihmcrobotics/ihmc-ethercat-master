package us.ihmc.etherCAT.slaves;

import us.ihmc.etherCAT.slaves.EasyCATSlave;

import java.io.IOException;

public class EasyCATActuatorLowerJoint extends EasyCATSlave
{

   protected int[] frameData = new int[32];
   protected double currentRotationRaw1 = 0;
   protected double currentRotationRaw2 = 0;
   protected double currentRotationProcessed1 = 0;
   protected double currentRotationProcessed2 = 0;

   protected double actuatorTemp = 0;
   protected int dataFlipBit = 0;

   public EasyCATActuatorLowerJoint(int alias, int ringPosition) throws IOException
   {
      super(alias, ringPosition);
   }

   protected void processDataFromLowerJoint()
   {

      getTransmitBytes(frameData, 0, 31);

      processActuatorTemperature();
      processEncoders();

      dataFlipBit = frameData[10];
   }

   protected void processEncoders()
   {
   }

   protected void processActuatorTemperature()
   {
   }
   
   public double getActuatorTemperatureInC()
   {
      return actuatorTemp;
   }
   
   public double getCurrentRotation1()
   {
      return currentRotationProcessed1;
   }
   
   public double getCurrentRotation2()
   {
      return currentRotationProcessed2;
   }

   public int getDataFlipBit()
   {
      return dataFlipBit;
   }
}
