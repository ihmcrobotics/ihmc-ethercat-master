package us.ihmc.etherCAT.master.exception;

import us.ihmc.etherCAT.master.Master;
import us.ihmc.etherCAT.master.Slave;

import java.util.Collection;

public class SlavesOfflineException extends SlaveCountException
{
   public SlavesOfflineException(int currentSlaveCount, int registeredSlaveCount, Collection<Slave> slaves, Master master)
   {
      super(currentSlaveCount, registeredSlaveCount, slaves, master);
   }

   @Override
   public String getMessage()
   {
      StringBuilder slaveStringBuilder = new StringBuilder();

      for (Slave slave : getSlaves())
      {
         slaveStringBuilder.append(slave.toString()).append(", ");
      }

      return "Not all registeredSlaves are online" + (master.isRequireAllSlaves() ? " and requireAllSlaves is true" : "") + ".\n[" + getCurrentSlaveCount()
             + " / " + getRegisteredSlaveCount() + " slaves online.\nOffline slaves: [" + slaveStringBuilder + "]";
   }
}
