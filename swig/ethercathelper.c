#include "oshw.h"
#include "osal.h"
#include "ethercattype.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercathelper.h"

#include <stdlib.h>
#include <stdio.h>

#ifdef __linux__
#include <linux/types.h>
#include <linux/ethtool.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <linux/sockios.h>
#include <linux/netlink.h>
#include <sys/ioctl.h>
#include <string.h>
#include <net/if.h>
#endif

ec_slavet             ihmc_ecslave[EC_MAXSLAVE];
int                   ihmc_ecslavecount;
ec_groupt             ihmc_ecgroup[EC_MAXGROUP];

static uint8          ihmc_ecesibuf[EC_MAXEEPBUF];
static uint32         ihmc_ecesimap[EC_MAXEEPBITMAP];
static ec_eringt      ihmc_ecelist;
static ec_idxstackT   ihmc_ecidxstack;

static ec_SMcommtypet ihmc_ecSMcommtype;
static ec_PDOassignt  ihmc_ecPDOassign;
static ec_PDOdesct    ihmc_ecPDOdesc;

static ec_eepromSMt   ihmc_ecSM;
static ec_eepromFMMUt ihmc_ecFMMU;
boolean                 ihmc_EcatError = FALSE;

int64                 ihmc_ecDCtime;

ecx_portt               ihmc_ecx_port;
ecx_redportt            ihmc_ecx_redport;

ecx_contextt  ihmc_ecx_context = {
    &ihmc_ecx_port,       // .port          =
    &ihmc_ecslave[0],    // .slavelist     =
    &ihmc_ecslavecount,  // .slavecount    =
    EC_MAXSLAVE,     // .maxslave      =
    &ihmc_ecgroup[0],    // .grouplist     =
	EC_MAXGROUP,     // .maxgroup      =
    &ihmc_ecesibuf[0],   // .esibuf        =
    &ihmc_ecesimap[0],   // .esimap        =
    0,               // .esislave      =
    &ihmc_ecelist,       // .elist         =
    &ihmc_ecidxstack,    // .idxstack      =
    &ihmc_EcatError,      // .ecaterror     =
    0,               // .DCtO          =
    0,               // .DCl           =
    &ihmc_ecDCtime,      // .DCtime        =
    &ihmc_ecSMcommtype,  // .SMcommtype    =
    &ihmc_ecPDOassign,   // .PDOassign     =
    &ihmc_ecPDOdesc,     // .PDOdesc       =
    &ihmc_ecSM,          // .eepSM         =
    &ihmc_ecFMMU,        // .eepFMMU       =
    NULL             // .FOEhook()
};


ecx_contextt* ecx_create_context()
{  
/*
    ecx_contextt* ihmc_ecx_context = (ecx_contextt*) malloc(sizeof(ecx_contextt));
    
	ec_slavet             *ihmc_ecslave = malloc(sizeof(*ihmc_ecslave) * EC_MAXSLAVE);
	int                   *ihmc_ecslavecount = malloc(sizeof(*ihmc_ecslavecount));
	ec_groupt             *ihmc_ecgroup = malloc(sizeof(*ihmc_ecgroup) * EC_MAXGROUP);
	                      
	uint8                 *ihmc_ecesibuf = malloc(sizeof(*ihmc_ecesibuf) * EC_MAXEEPBUF);
	uint32                *ihmc_ecesimap = malloc(sizeof(*ihmc_ecesimap) * EC_MAXEEPBITMAP);
	ec_eringt             *ihmc_ecelist = malloc(sizeof(*ihmc_ecelist));
	ec_idxstackT          *ihmc_ecidxstack = malloc(sizeof(*ihmc_ecidxstack));
	                      
	ec_SMcommtypet        *ihmc_ecSMcommtype = malloc(sizeof(*ihmc_ecSMcommtype));
	ec_PDOassignt         *ihmc_ecPDOassign = malloc(sizeof(*ihmc_ecPDOassign));
	ec_PDOdesct           *ihmc_ecPDOdesc = malloc(sizeof(*ihmc_ecPDOdesc));
	                      
	ec_eepromSMt          *ihmc_ecSM = malloc(sizeof(*ihmc_ecSM));
	ec_eepromFMMUt        *ihmc_ecFMMU = malloc(sizeof(*ihmc_ecFMMU));
	boolean               *ihmc_EcatError = malloc(sizeof(*ihmc_EcatError));
	                      
	int64                 *ihmc_ecDCtime = malloc(sizeof(*ihmc_ecDCtime));
	                      
	ecx_portt             *ihmc_ecx_port = malloc(sizeof(*ihmc_ecx_port));
	ecx_redportt          *ihmc_ecx_redport = malloc(sizeof(*ihmc_ecx_redport));
	 
    
   ihmc_ecx_context->port    = ihmc_ecx_port;
   ihmc_ecx_context->slavelist = ihmc_ecslave;
   ihmc_ecx_context->slavecount = ihmc_ecslavecount;
   ihmc_ecx_context->maxslave = EC_MAXSLAVE;
   ihmc_ecx_context->grouplist =  ihmc_ecgroup;
   ihmc_ecx_context->maxgroup = EC_MAXGROUP;
   ihmc_ecx_context->esibuf = ihmc_ecesibuf;
   ihmc_ecx_context->esimap = ihmc_ecesimap;
   ihmc_ecx_context->esislave =    0;               
   ihmc_ecx_context->elist = ihmc_ecelist;
   ihmc_ecx_context->idxstack = ihmc_ecidxstack;
   ihmc_ecx_context->ecaterror = ihmc_EcatError;
   ihmc_ecx_context->DCtO =    0;          
   ihmc_ecx_context->DCl =    0;     
   ihmc_ecx_context->DCtime = ihmc_ecDCtime;
   ihmc_ecx_context->SMcommtype = ihmc_ecSMcommtype;
   ihmc_ecx_context->PDOassign = ihmc_ecPDOassign;
   ihmc_ecx_context->PDOdesc = ihmc_ecPDOdesc;
   ihmc_ecx_context->eepSM = ihmc_ecSM;
   ihmc_ecx_context->eepFMMU = ihmc_ecFMMU;
   ihmc_ecx_context->FOEhook = NULL;
    */
    return &ihmc_ecx_context;
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
    free(ecx_context);
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

ec_smt* ecx_sm(ec_slavet* slave, uint32 sm)
{
	if(sm >= EC_MAXSM)
	{
		return NULL;
	}
	
	return &(slave->SM[sm]);
}

uint8 ecx_smtype(ec_slavet* slave, uint32 sm)
{
	if(sm >= EC_MAXSM)
	{
		return 0;
	}
	
	return slave->SMtype[sm];
}

ec_fmmut* ecx_fmmu(ec_slavet* slave, uint32 fmmu)
{
	if(fmmu >= EC_MAXFMMU)
	{
		return NULL;
	}
	
	return &(slave->FMMU[fmmu]);
}

int32_t ecx_inputoffset(ec_slavet* slave, void* buffer)
{
    return ((uint64_t) slave->inputs) - ((uint64_t) buffer);
}

char* ecx_oelist_name(ec_OElistt* OElist, int32 index)
{
	return OElist->Name[index];
}


char* ecx_odlist_name(ec_ODlistt* ODlist, int32 index)
{
	return ODlist->Name[index];
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

int ecx_reconfig_slave_to_preop(ecx_contextt *context, uint16 slave, int timeout)
{
   int state, nSM;
   uint16 configadr;

   configadr = context->slavelist[slave].configadr;
   if (ecx_FPWRw(context->port, configadr, ECT_REG_ALCTL, htoes(EC_STATE_INIT) , timeout) <= 0)
   {
      return 0;
   }
   state = 0;
   ecx_eeprom2pdi(context, slave); /* set Eeprom control to PDI */
   /* check state change init */
   state = ecx_statecheck(context, slave, EC_STATE_INIT, EC_TIMEOUTSTATE);
   if(state == EC_STATE_INIT)
   {
      /* program all enabled SM */
      for( nSM = 0 ; nSM < EC_MAXSM ; nSM++ )
      {
         if (context->slavelist[slave].SM[nSM].StartAddr)
         {
            ecx_FPWR(context->port, configadr, ECT_REG_SM0 + (nSM * sizeof(ec_smt)),
               sizeof(ec_smt), &context->slavelist[slave].SM[nSM], timeout);
         }
      }
      ecx_FPWRw(context->port, configadr, ECT_REG_ALCTL, htoes(EC_STATE_PRE_OP) , timeout);
      state = ecx_statecheck(context, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE); /* check state change pre-op */
   }

   return state;
}

int ecx_reconfig_slave_to_safeop(ecx_contextt *context, uint16 slave, int timeout)
{
   int state, FMMUc;
   uint16 configadr;

   configadr = context->slavelist[slave].configadr;

   state = ecx_statecheck(context, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE); /* check state change pre-op */
   if( state == EC_STATE_PRE_OP)
      {
         ecx_FPWRw(context->port, configadr, ECT_REG_ALCTL, htoes(EC_STATE_SAFE_OP) , timeout); /* set safeop status */
         state = ecx_statecheck(context, slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE); /* check state change safe-op */
         /* program configured FMMU */
         for( FMMUc = 0 ; FMMUc < context->slavelist[slave].FMMUunused ; FMMUc++ )
         {
            ecx_FPWR(context->port, configadr, ECT_REG_FMMU0 + (sizeof(ec_fmmut) * FMMUc),
               sizeof(ec_fmmut), &context->slavelist[slave].FMMU[FMMUc], timeout);
         }
      }
}

int32_t ecx_FOEread_java_helper(ecx_contextt *context, uint16 slave, char *filename, uint32 password, int size, void *p, int timeout)
{
	int psize = size;

	int res = ecx_FOEread(context, slave, filename, password, &psize, p, timeout);
	if(res > 0)
	{
		return psize;
	}
	else
	{
	 	return res;
	}
}

uint8 ecx_setup_socket_fast_irq(char *iface)
{
#ifdef __linux__
	struct ethtool_coalesce ecoal;
    struct ethtool_drvinfo drvinfo;	
	int err;
	
	struct ifreq ifr;
	memset(&ifr, 0, sizeof(ifr));
	strcpy(ifr.ifr_name, iface);
	
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0)
		fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_GENERIC);
	if (fd < 0) {
		perror("Cannot get control socket");
		return 70;
	}
	
    drvinfo.cmd = ETHTOOL_GDRVINFO;
    ifr.ifr_data = (char*)&drvinfo;
    err = ioctl(fd,SIOCETHTOOL, &ifr);
    if(err)
    {
        return 73;
    }
	
	
	
	ecoal.cmd = ETHTOOL_GCOALESCE;
	ifr.ifr_data = (char*)&ecoal;
	err = ioctl(fd, SIOCETHTOOL, &ifr);
	if (err) {
		return 76;
	}

    if(strcmp(drvinfo.driver, "igb") == 0)
    {
        ecoal.rx_coalesce_usecs = 0;
        ecoal.tx_coalesce_usecs = 0;
    }
    else if(strcmp(drvinfo.driver, "tg3") == 0)
    {
        ecoal.rx_coalesce_usecs = 1;
        ecoal.rx_max_coalesced_frames = 1;
        ecoal.tx_coalesce_usecs = 1;
        ecoal.tx_max_coalesced_frames = 1;
    }
    else
    {
        ecoal.rx_coalesce_usecs = 0;
        ecoal.rx_max_coalesced_frames = 1;
        ecoal.tx_coalesce_usecs = 0;
        ecoal.tx_max_coalesced_frames = 1;
    }

	
	ecoal.cmd = ETHTOOL_SCOALESCE;
	ifr.ifr_data = (char*)&ecoal;
	err = ioctl(fd, SIOCETHTOOL, &ifr);
	if (err) {
		return 81;
	}
	return 1;
#else
	return 10;
#endif
}