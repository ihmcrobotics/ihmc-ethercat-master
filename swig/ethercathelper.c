#include "oshw.h"
#include "osal.h"
#include "ethercattype.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatcoe.h"
#include "ethercathelper.h"

#include <stdlib.h>

ecx_contextt* ecx_create_context()
{  
    ecx_contextt* ecx_context = (ecx_contextt*) malloc(sizeof(ecx_contextt));
    
    
    ecx_context->port    = (ecx_portt*) malloc(sizeof(ecx_portt));
    ecx_context->slavelist = (ec_slavet*) malloc(sizeof(ec_slavet) * EC_MAXSLAVE);
    ecx_context->slavecount =     (int*) malloc(sizeof(int));  // .slavecount    =
    ecx_context->maxslave = EC_MAXSLAVE;     // .maxslave      =
    ecx_context->grouplist =    (ec_groupt*) malloc(sizeof(ec_groupt) * EC_MAXGROUP);    // .grouplist     =
    ecx_context->maxgroup = EC_MAXGROUP,     // .maxgroup      =
    ecx_context->esibuf = (uint8*) malloc(sizeof(uint8) * EC_MAXEEPBUF);   // .esibuf        =
    ecx_context->esimap = (uint32*) malloc(sizeof(uint32) * EC_MAXEEPBITMAP);   // .esimap        =
    ecx_context->esislave =    0;               // .esislave      =
    ecx_context->elist = (ec_eringt*) malloc(sizeof(ec_eringt)); //    &ec_elist,       // .elist         =
    ecx_context->idxstack = (ec_idxstackT*) malloc(sizeof(ec_idxstackT)); //   &ec_idxstack,    // .idxstack      =
    ecx_context->ecaterror = (boolean*) malloc(sizeof(boolean));      // .ecaterror     =
    ecx_context->DCtO =    0;               // .DCtO          =
    ecx_context->DCl =    0;               // .DCl           =
    ecx_context->DCtime = (int64*) malloc(sizeof(int64));  //  &ec_DCtime,      // .DCtime        =
    ecx_context->SMcommtype = (ec_SMcommtypet*) malloc(sizeof(ec_SMcommtypet)); //    &ec_SMcommtype,  // .SMcommtype    =
    ecx_context->PDOassign = (ec_PDOassignt*) malloc(sizeof(ec_PDOassignt));
    ecx_context->PDOdesc = (ec_PDOdesct*) malloc(sizeof(ec_PDOdesct));
    ecx_context->eepSM = (ec_eepromSMt*) malloc(sizeof(ec_eepromSMt));
    ecx_context->eepFMMU = (ec_eepromFMMUt*) malloc(sizeof(ec_eepromFMMUt));
    ecx_context->FOEhook = NULL;
    
    
    return ecx_context;
}

void ecx_destroy_context(ecx_contextt* ecx_context)
{
    free(ecx_context->port);
    free(ecx_context->slavelist);
    free(ecx_context->slavecount);
    free(ecx_context->grouplist);
    free(ecx_context->esibuf);
    free(ecx_context->esimap);
    free(ecx_context->elist);
    free(ecx_context->idxstack);
    free(ecx_context->ecaterror);
    free(ecx_context->DCtime);
    free(ecx_context->SMcommtype);
    free(ecx_context->PDOassign);
    free(ecx_context->PDOdesc);
    free(ecx_context->eepSM);
    free(ecx_context->eepFMMU);
}

boolean ecx_ecaterror(ecx_contextt* ecx_context)
{
	return *(ecx_context->ecaterror);
}

int64 ecx_dcTime(ecx_contextt* context)
{
	return *(context->DCtime);
}

int ecx_slavecount(ecx_contextt* context)
{
    return *(context->slavecount);
}

ec_slavet* ecx_slave(ecx_contextt* context, uint32 slave)
{
    if(slave > ecx_slavecount(context))
    {
        return NULL;
    }
    
    return &(context->slavelist[slave]);
}

int32_t ecx_inputoffset(ec_slavet* slave, void* buffer)
{
    return ((uint64_t) slave->inputs) - ((uint64_t) buffer);
}


int32_t ecx_outputoffset(ec_slavet* slave, void* buffer)
{
    return ((uint64_t) slave->outputs) - ((uint64_t) buffer);
}

int ecx_SDOread_java_helper(ecx_contextt *context, uint16 slave, uint16 index, uint8 subindex,
                      boolean CA, int size, void *p, int timeout)
{
	return ecx_SDOread(context, slave, index, subindex, CA, &size, p, timeout);
	
}