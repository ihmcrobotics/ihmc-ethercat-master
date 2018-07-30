
#ifndef _EC_ECATHELPER_H
#define _EC_ECATHELPER_H

#ifdef __cplusplus
extern "C"
{
#endif

ecx_contextt* ecx_create_context();
void ecx_destroy_context(ecx_contextt* context);
ec_slavet* ecx_slave(ecx_contextt* context, uint32 slave);
int ecx_slavecount(ecx_contextt* context);
int32_t ecx_inputoffset(ec_slavet* slave, void* buffer);
int32_t ecx_outputoffset(ec_slavet* slave, void* buffer);
boolean ecx_ecaterror(ecx_contextt* ecx_context);
int64 ecx_dcTime(ecx_contextt* context);
int ecx_SDOread_java_helper(ecx_contextt *context, uint16 slave, uint16 index, uint8 subindex,
                      boolean CA, int size, void *p, int timeout);
int ecx_reconfig_slave_to_preop(ecx_contextt *context, uint16 slave, int timeout);
int ecx_reconfig_slave_to_safeop(ecx_contextt *context, uint16 slave, int timeout);
ec_smt* ecx_sm(ec_slavet* slave, uint32 sm);
uint8 ecx_smtype(ec_slavet* slave, uint32 sm);
ec_fmmut* ecx_fmmu(ec_slavet* slave, uint32 fmmu);
char* ecx_oelist_name(ec_OElistt* OElist, int32 index);
char* ecx_odlist_name(ec_ODlistt* ODlist, int32 index);


int32_t ecx_FOEread_java_helper(ecx_contextt *context, uint16 slave, char *filename, uint32 password, int size, void *p, int timeout);

void ecx_writembxconfig(ecx_contextt *context, ec_slavet* ec_slave);


uint8 ecx_setup_socket_fast_irq(char *iface);


#ifdef __cplusplus
}
#endif

#endif 