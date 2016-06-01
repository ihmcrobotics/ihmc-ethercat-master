package us.ihmc.soem.slaves;

public enum ElmoModeOfOperation
{
   CAN_ENCODER_MODE(-3),
   NO_MODE(0),
   PROFILE_POSITION_MODE(1),
   VELOCITY(2), // Not supported
   PROFILED_VELOCITY_MODE(3),
   TORQUE_PROFILED_MODE(4),
   HOMING_MODE(6),
   INTERPOLATED_POSITION_MODE(7),
   CYCLIC_SYNCHRONOUS_POSITION(8), // DSP402
   CYCLIC_SYNCHRONOUS_VELOCITY(9), // DSP402
   CYCLIC_SYNCHRONOUS_TORQUE(10); // DSP402
   
   private final byte modeByte; 
   private ElmoModeOfOperation(int mode)
   {
      this.modeByte = (byte) mode;
   }
   
   public byte getMode()
   {
      return modeByte;
   }
}
