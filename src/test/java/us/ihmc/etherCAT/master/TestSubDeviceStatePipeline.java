package us.ihmc.etherCAT.master;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Test;

import us.ihmc.etherCAT.master.pipeline.LightWeightPipelineExecutor;

public class TestSubDeviceStatePipeline
{
   @Test
   public void testSubDevicePipeline()
   {

      TestSubdevice subDevice = new TestSubdevice();
      
      ReadSDO readSdo = new ReadSDO(subDevice, 0, 0, 8);
      subDevice.registerSDO(readSdo);
      WriteSDO writeSdo = new WriteSDO(subDevice, 0, 0, 8);
      subDevice.registerSDO(writeSdo);
      
      Master master = new Master("/dev/null")
      {
         @Override
         public int getActualWorkingCounter()
         {
            return subDevice.state.ordinal();
         }
      };
      
      master.setReadRXErrorStatistics(true);

      SubDeviceStatePipeline pipeline = new SubDeviceStatePipeline(master, subDevice);

      LightWeightPipelineExecutor executor = new LightWeightPipelineExecutor(pipeline.getPipelineBin());

      long startTime = System.nanoTime();

      for (int i = 0; i < 100; i++)
      {
         long runtime = System.nanoTime() - startTime;
         
         readSdo.requestNewData();
         writeSdo.write(10.0);

         executor.execute(runtime);
         
         
         
         System.out.println(subDevice.state);
      }
   }

   private class TestSubdevice extends Slave
   {
      private State state = State.OFFLINE;

      public TestSubdevice()
      {
         super(0, 0, 0, 0);
      }

      @Override
      void doEtherCATStateControl(long runTime)
      {
         if(state != State.OP)
         {
            state = state.values()[state.ordinal() + 1];
         }
      }
      
      @Override
      boolean updateEtherCATState()
      {
         return true;
      }
      
      @Override
      boolean updateRXTXStats()
      {
         return true;
      }
      
      @Override
      State getHouseholderState()
      {
         return state;
      }

      @Override
      public int readSDOToBuffer(int index, int subIndex, int size, ByteBuffer sdoBuffer)
      {
         return 1;
      }

      @Override
      public int writeSDO(int index, int subindex, ByteBuffer buffer)
      {
         return 1;
      }
   }
}
