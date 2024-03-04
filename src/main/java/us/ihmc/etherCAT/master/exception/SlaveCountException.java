package us.ihmc.etherCAT.master.exception;

import us.ihmc.etherCAT.master.Master;
import us.ihmc.etherCAT.master.Slave;

import java.io.IOException;
import java.util.Collection;

public abstract class SlaveCountException extends IOException
{
   private final int currentSlaveCount;
   private final int registeredSlaveCount;
   private final Collection<Slave> slaves;
   protected final Master master;

   public SlaveCountException(int currentSlaveCount, int registeredSlaveCount, Collection<Slave> slaves, Master master)
   {
      this.currentSlaveCount = currentSlaveCount;
      this.registeredSlaveCount = registeredSlaveCount;
      this.slaves = slaves;
      this.master = master;
   }

   public abstract String getMessage();

   public int getCurrentSlaveCount()
   {
      return currentSlaveCount;
   }

   public int getRegisteredSlaveCount()
   {
      return registeredSlaveCount;
   }

   public Collection<Slave> getSlaves()
   {
      return slaves;
   }
}
