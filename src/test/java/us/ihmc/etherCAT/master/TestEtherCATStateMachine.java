package us.ihmc.etherCAT.master;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Test;

import us.ihmc.etherCAT.master.Slave.State;

public class TestEtherCATStateMachine
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
         long samples = EtherCATStateMachine.MINIMUM_JITTER_SAMPLES - 3;
         
         @Override
         public int getActualWorkingCounter()
         {
            return subDevice.state.ordinal();
         }
         
         @Override
         public int getExpectedWorkingCounter()
         {
            return State.OP.ordinal();
         }
         
         @Override
         public long getJitterEstimate()
         {
            return 1000;
         }
         
         @Override
         public long getJitterSamples()
         {
            return samples++;
         }
      };
      
      master.setReadRXErrorStatistics(true);
      master.enableDC(1000000);
      

      
      EtherCATStateMachine stateMachine = new EtherCATStateMachine(master);
      stateMachine.setSubdevices(new Slave[] { subDevice });

      for (int i = 0; i < 100; i++)
      {
         System.out.println("----");
         
         readSdo.requestNewData();
         writeSdo.write(10.0);
         
         subDevice.updateStateVariables();

         if(readSdo.isValid())
         {
            System.out.println("Read SDO Is valid");
         }
         
         if(writeSdo.isValid())
         {
            System.out.println("Write SDO Is valid");
         }
         

         stateMachine.runOnce();
         if(i == 92)
         {
            subDevice.state  = State.SAFE_OPERR;
         }
         
         
         System.out.println(stateMachine.getExecutor().getLastExecutedTaskIndex() + ": " + stateMachine.getExecutor().getLastExectutedTaskName());
         
         System.out.println(subDevice.getHouseholderState());
      }
      
      assertEquals(subDevice.state, State.OP);
   }
   

   private class TestSubdevice extends Slave
   {
      private State state = State.OFFLINE;
      private State retState = State.OFFLINE;

      public TestSubdevice()
      {
         super(0, 0, 0, 0);
      }

      @Override
      void doEtherCATStateControl(long runTime)
      {
         if(state != State.OP)
         {
            state = State.values()[state.ordinal() + 1];
         }
      }
      
      @Override
      boolean updateEtherCATState()
      {
         retState = state;
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
         return retState;
      }

      @Override
      public int readSDOToBuffer(int index, int subIndex, int size, ByteBuffer sdoBuffer)
      {
         System.out.println("SDO Read command executed");
         return 1;
      }

      @Override
      public int writeSDO(int index, int subindex, ByteBuffer buffer)
      {
         System.out.println("SDO Write command executed");
         return 1;
      }
   }
}
