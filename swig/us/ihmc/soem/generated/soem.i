%module soem

%include "various.i"
%apply unsigned char *NIOBUFFER { void* };

%include "carrays.i"


%javaconst(1);
%{
    #define EC_VER2
	#include "osal_defs.h"
	#include "ethercat.h"
	#include "ethercathelper.h"	
%}


%define EC_VER2

%include "stdint.i"

#define PACKED
#define PACKED_BEGIN
#define PACKED_END

%array_functions(uint16, uint16Array);
%array_functions(uint8, uint8Array);

%include "osal_defs.h"
%include "osal.h"

%include "ethercattype.h"
%include "nicdrv.h"
%include "ethercatbase.h"
%include "ethercatmain.h"
%include "ethercatdc.h"
%include "ethercatcoe.h"
%include "ethercatfoe.h"
%include "ethercatsoe.h"
%include "ethercatconfig.h"
%include "ethercatprint.h"
%include "ethercathelper.h"



%enddef EC_VER2
