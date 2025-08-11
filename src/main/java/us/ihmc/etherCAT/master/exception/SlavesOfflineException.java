package us.ihmc.etherCAT.master.exception;

import us.ihmc.etherCAT.master.Master;
import us.ihmc.etherCAT.master.Slave;

import java.util.ArrayList;
import java.util.List;

public class SlavesOfflineException extends SlaveCountException
{
   public SlavesOfflineException(int currentSlaveCount, int registeredSlaveCount, List<Slave> slaves, Master master)
   {
      super(currentSlaveCount, registeredSlaveCount, slaves, master);
   }

   @Override
   public String getMessage()
   {
      List<String> slaveNames = new ArrayList<>();
      for (Slave slave : getSlaves())
         slaveNames.add(slave.getName());

      String slavesListString = String.join(",", slaveNames);

      return "Not all registeredSlaves are online" + (master.isRequireAllSlaves() ? " and requireAllSlaves is true" : "") + ".\n[" + getCurrentSlaveCount()
             + " / " + getRegisteredSlaveCount() + "] slaves online.\nOffline slaves: [" + slavesListString + "]";
   }
}
