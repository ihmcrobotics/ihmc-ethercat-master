package us.ihmc.etherCAT.master;

import java.util.List;

/**
 * Interface for commonly used master functions. Useful to switch between EtherCATRealtimeThread and Master.
 * 
 * @author jesper
 *
 */
public interface MasterInterface
{

   public void setMaximumExecutionJitter(long jitterInNanoseconds);

   public long getJitterEstimate();

   public void registerSlave(Slave slave);

   public void registerSDO(SDO sdo);

   public List<Slave> getSlaves();

   void setEtherCATReceiveTimeout(int timeout);
   
   void disableRecovery();
}
