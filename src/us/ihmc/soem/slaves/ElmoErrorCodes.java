package us.ihmc.soem.slaves;

public class ElmoErrorCodes
{

   private static final String[] EC = new String[246];

   static
   {
      EC[0] = "No error";
      EC[1] = "Do not Update";
      EC[2] = "Bad Command";
      EC[3] = "Bad Index";
      EC[4] = "PAL does not support this sensor";
      EC[7] = "Mode cannot be started - bad initialization data";
      EC[9] = "CAN message HW buffer was overrun";
      EC[10] = "Cannot be used by PDO";
      EC[11] = "Cannot write to flash memory";
      EC[13] = "Cannot reset communication - UART is busy";
      EC[16] = "Array '[ ]' is expected, or empty expression in array";
      EC[17] = "Format of UL command is not valid - check the command definition";
      EC[19] = "Command syntax error";
      EC[20] = "Bad Set Point sending order";
      EC[21] = "Operand Out of Range";
      EC[23] = "Command cannot be assigned";
      EC[26] = "Profiler mode not supported in this unit mode (UM)";
      EC[27] = "Bad ECAM setting";
      EC[28] = "Out Of Limit Range";
      EC[33] = "Bad sensor setting";
      EC[34] = "There is a conflict with another command";
      EC[35] = "Max bus voltage (BV) or max current (MC) is not valid";
      EC[36] = "Commutation method (CA[17]) or commutation table does not fit to sensor";
      EC[37] = "Two Or More Hall sensors are defined to the same place";
      EC[38] = "PORT C setting is incorrect";
      EC[40] = "In Wizard Experiment!";
      EC[42] = "No Such Label";
      EC[45] = "An Attempt to read a write only command";
      EC[47] = "Program does not exist or not Compiled";
      EC[48] = "Motor cold not start - fault reason in CD";
      EC[51] = "Inhibit OR Abort inputs are active, Cannot start motor";
      EC[54] = "Bad Data Base";
      EC[57] = "Motor Must be Off";
      EC[58] = "Motor Must be On";
      EC[60] = "Bad Unit Mode";
      EC[61] = "Data Base Reset";
      EC[62] = "Socket change not allowed";
      EC[67] = "Recorder Is Busy";
      EC[68] = "Required profiler mode is not supported";
      EC[69] = "Recorder Usage Error";
      EC[70] = "Recorder data Invalid";
      EC[71] = "Homing is busy";
      EC[74] = "Bad profile database, see 0x2081 for object number (EE[2])";
      EC[75] = "Download is in progress";
      EC[76] = "Error mapping is not allowed";
      EC[78] = "Out of Program Range";
      EC[79] = "Sensor setting error";
      EC[81] = "Download failed see specific error in EE[3]";
      EC[82] = "Program Is Running";
      EC[83] = "Command is not permitted in a program.";
      EC[85] = "STO is not active";
      EC[87] = "Hall sensed with illegal value";
      EC[94] = "Not allowed while Error mapping";
      EC[97] = "RS232 receive buffer overflow";
      EC[98] = "Cannot measure current offsets";
      EC[99] = "The sensor does not support this command";
      EC[100] = "The requested PWM value is not supported";
      EC[101] = "Absolute encoder setting problem";
      EC[102] = "Output Compare is busy";
      EC[103] = "Output Compare Sensor Is Not QUAD Encoder";
      EC[104] = "Output Compare Table Length OR Data";
      EC[105] = "Speed loop KP out of range";
      EC[107] = "Encoder emulation parameter is out of range";
      EC[108] = "Encoder emulation in progress";
      EC[110] = "Too long number";
      EC[122] = "Motion mode is not supported or with initialization conflict";
      EC[123] = "Profiler queue is full";
      EC[125] = "Personality not loaded";
      EC[126] = "User Program failed - variable out of program size";
      EC[128] = "Bad variable index in database";
      EC[129] = "Variable is not an array";
      EC[130] = "Variable name does not exist";
      EC[131] = "Cannot record local variable";
      EC[132] = "Variable is an array";
      EC[133] = "Number of function input arguments is not as expected";
      EC[134] = "Cannot run local label/function with the XQ command";
      EC[135] = "Frequency identification failed";
      EC[136] = "Not a number";
      EC[138] = "Position Interpolation buffer underflow";
      EC[139] = "The number of break points exceeds maximal number";
      EC[140] = "An attempt to set/clear break point at the not relevant line";
      EC[142] = "Checksum of data is not correct";
      EC[144] = "Numeric Stack underflow";
      EC[145] = "Numeric stack overflow";
      EC[147] = "Executable command within math expression";
      EC[148] = "Nothing in the expression";
      EC[151] = "Parentheses mismatch";
      EC[152] = "Bad operand type";
      EC[153] = "Overflow in a numeric operator";
      EC[154] = "Address is out of data memory segment";
      EC[155] = "Beyond stack range";
      EC[156] = "Bad op-code";
      EC[158] = "Out of flash memory range";
      EC[159] = "Flash memory verification error";
      EC[161] = "Program is not halted";
      EC[163] = "Not enough space in program data segment";
      EC[165] = "An attempt to access flash memory while busy";
      EC[166] = "Out Of Modulo Range";
      EC[168] = "Speed too large to start motor";
      EC[169] = "Time out using peripheral.(overflow or busy)";
      EC[170] = "Cannot erase sector in flash memory";
      EC[171] = "Cannot read from flash memory";
      EC[172] = "Cannot write to flash memory";
      EC[173] = "Executable area of program is too large";
      EC[174] = "Program has not been loaded";
      EC[175] = "Cannot write program checksum - clear program (CP)";
      EC[176] = "User code, variables and functions are too large";
      EC[177] = "Capture/Compare conversion error in analog encoder. Require known quad location";
      EC[178] = "CAN bus off";
      EC[179] = "Consumer HB event";
      EC[180] = "DF is not supported in this communication type";
      EC[181] = "Writing to Flash program area, failed";
      EC[182] = "PAL Burn Is In Process or no PAL is burned";
      EC[184] = "Capture option already used by other operation";
      EC[185] = "This element may be modified only when interpolation is not active";
      EC[186] = "Interpolation queue is full";
      EC[187] = "Incorrect Interpolation sub-mode";
      EC[188] = "Gantry slave is disabled";
      EC[189] = "CAN message was lost, software buffer overflow";
      EC[200] = "Main Feedback error refer to: EE[1]";
      EC[201] = "Commutation sequence failed";
      EC[202] = "Encoder - Hall sensor mismatch refer to: XP[7]";
      EC[203] = "Current limit was exceeded";
      EC[204] = "External inhibit input detected";
      EC[205] = "AC Fail: loss of phase";
      EC[206] = "Digital Hall run too fast or disconnected";
      EC[207] = "Speed error limit exceeded refer to: ER[2]";
      EC[208] = "Position error limit exceeded value of ER[3]";
      EC[209] = "Cannot start motor because of bad database refer to: CD";
      EC[210] = "Bad ECAM table";
      EC[216] = "Cannot find zero position without DHalls";
      EC[217] = "Over Speed violation refer to: HL[2]";
      EC[221] = "Motor stuck";
      EC[222] = "Out of position limits refer to: HL[3] or LL[3]";
      EC[223] = "Numerical overflow";
      EC[224] = "Gantry slave is not enabled";
      EC[229] = "Cannot start motor because of internal problem";
      EC[233] = "Under voltage protection";
      EC[235] = "Overvoltage protection";
      EC[237] = "Safety switch";
      EC[241] = "Short protection";
      EC[243] = "Over temperature protection";
   }
   
   
   public static String errorCodeToString(int ec)
   {
      if(ec < 0 || ec >= EC.length)
      {
         return null;
      }
      
      return EC[ec];
   }
}
