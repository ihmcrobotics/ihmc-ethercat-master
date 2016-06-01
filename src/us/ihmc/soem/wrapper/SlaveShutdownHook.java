package us.ihmc.soem.wrapper;

public interface SlaveShutdownHook
{
   /**
    * 
    * Gets called cyclically by the master shutdown hook on termination until hasShutdown() returns true;
    * 
    */
   public void shutdown();
   
   /**
    * Gets called cyclically by the master to check if the slave has shutdown
    * 
    * @return true when the slave finished shutting down
    */
   public boolean hasShutdown();
}
