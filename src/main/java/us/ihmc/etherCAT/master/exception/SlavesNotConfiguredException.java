package us.ihmc.etherCAT.master.exception;

import us.ihmc.etherCAT.master.Master;
import us.ihmc.etherCAT.master.Slave;

import java.util.Collection;

public class SlavesNotConfiguredException extends SlaveCountException
{
   public SlavesNotConfiguredException(int currentSlaveCount, int registeredSlaveCount, Collection<Slave> slaves, Master master)
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

      return "Not all slaves are configured" + (master.isRequireAllSlaves() ? " and requireAllSlaves is true" : "") + ".\n[" + getCurrentSlaveCount()
             + " / " + getRegisteredSlaveCount() + " slaves online.\nInvalid slaves: [" + slaveStringBuilder + "]";
   }
}
