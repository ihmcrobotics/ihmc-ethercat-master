package us.ihmc.etherCAT.master.householder;

import us.ihmc.etherCAT.master.Master;
import us.ihmc.etherCAT.master.Slave;

public class SubDeviceStatePipeline
{
   private final Master mainDevice;
   private final Slave subDevice;
   
   
   private boolean hasReachedOp = false;
   private int previousWkc = 0;
   
   public SubDeviceStatePipeline(Master mainDevice, Slave subDevice)
   {
      this.mainDevice = mainDevice;
      this.subDevice = subDevice;
   }
   
   
   
}
