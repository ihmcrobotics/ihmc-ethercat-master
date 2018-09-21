package us.ihmc.etherCAT.slaves;

import us.ihmc.etherCAT.slaves.EasyCATSlave;

import java.io.IOException;

public class EasyCATActuatorLowerJoint extends EasyCATSlave
{

   protected int[] frameData = new int[32];
   protected int currentRotationRaw1 = 0;
   protected int currentRotationRaw2 = 0;
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
      currentRotationRaw1 = (short) (((frameData[0] & 0xFF) << 8 | (frameData[1] & 0xFF)));
      currentRotationRaw2 = (short) (((frameData[2] & 0xFF) << 8 | (frameData[3] & 0xFF)));
   }

   protected void processActuatorTemperature()
   {
      actuatorTemp = frameData[4];
   }

   public double getActuatorTemperatureInC()
   {
      return actuatorTemp;
   }

   public int getRawRotation1()
   {
      return currentRotationRaw1;
   }

   public int getRawRotation2()
   {
      return currentRotationRaw2;
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
