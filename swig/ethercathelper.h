
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

#ifdef __cplusplus
}
#endif

#endif 