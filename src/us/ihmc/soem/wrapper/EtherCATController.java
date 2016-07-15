package us.ihmc.soem.wrapper;


/**
 * Interface to signal the EtherCAT controller that the master is shutting down. 
 * 
 * @author Jesper Smith
 *
 */
public interface EtherCATController
{
   public void stopController();
   public void join() throws InterruptedException;
}
