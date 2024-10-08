
#include <string.h>
#include <stdlib.h>
#include "rhsp/module.h"
#include "rhsp/compiler.h"
#include "internal/arrayutils.h"
#include "internal/packet.h"
#include "internal/command.h"
#include "internal/module.h"
#include "internal/revhub.h"
#include <stdio.h>
static const RhspModuleInterface* getInterfaceByName(const RhspModuleInterfaceListInternal* list,
                                                     const char* interfaceName)
{
    for (size_t i = 0; i < list->numberOfInterfaces; i++)
    {
        //printf("testando %d", i);
        //printf("comparing %s to %s", list->interfaces[i].name, interfaceName);
        if (strcmp(interfaceName, list->interfaces[i].name) == 0)
        {
            return &list->interfaces[i];
        }
    }
    return NULL;
}

/**
 * Create the interface list if needed, then add this interface to it.
 * @param hub the hub to add this interface to
 * @param intf
 */
static void addInterface(RhspRevHubInternal* hub,
                         const RhspModuleInterface* intf)
{
    //printf("bing %p\n", hub->interfaceList);
    if (!hub->interfaceList)
    {
        //printf("tanuloo");
        hub->interfaceList = malloc(
                sizeof(RhspModuleInterfaceListInternal) + RHSP_MAX_NUMBER_OF_INTERFACES * sizeof(RhspModuleInterface));

        ((RhspModuleInterfaceListInternal*) hub->interfaceList)->numberOfInterfaces = 0;
    }
    RhspModuleInterfaceListInternal* list = (RhspModuleInterfaceListInternal*) hub->interfaceList;
    /* return early if the list is full, since we can't add an interface */
    if (list->numberOfInterfaces >= RHSP_MAX_NUMBER_OF_INTERFACES)
    {
        // if we had separate behavior for items already in the list,
        // we'd need to move this check after checking if the item
        // is present, since we won't add an item in that case, but
        // the behavior is the same for both, and this check is faster.
        return;
    }
    /* check whether interface is already added and return if we have already discovered interface */
    //printf("getinterface2");
    if (getInterfaceByName(list, intf->name))
    {
        return;
    }
    list->interfaces[list->numberOfInterfaces] = *intf;
    list->numberOfInterfaces++;
}

static uint16_t getInterfacePacketIdInternal(const RhspModuleInterfaceListInternal* interfaceList,
                                             const char* interfaceName,
                                             uint16_t functionNumber)
{
    if (interfaceList == NULL)
    {
        return RHSP_INTERFACE_INVALID_PACKET_ID;
    }
    //printf("antes do getbyname 2\n");
    //printf("comparar %s", interfaceName);
    const RhspModuleInterface* intf = getInterfaceByName(interfaceList, interfaceName);
    //printf("depois do getbyname 2\n");
    if (intf == NULL)
    {
        return RHSP_INTERFACE_INVALID_PACKET_ID;
    }
    if (functionNumber < intf->numberIDValues)
    {
        return intf->firstPacketID + functionNumber;
    }
    return RHSP_INTERFACE_INVALID_PACKET_ID;
}

int rhsp_getInterfacePacketID(RhspRevHub* hub,
                              const char* interfaceName,
                              uint16_t functionNumber,
                              uint16_t* packetID,
                              uint8_t* nackReasonCode)
{
    if (!hub || !interfaceName)
    {
        return RHSP_ERROR;
    }

    //printf("depouis do if\n");

    RhspRevHubInternal* internalHub = (RhspRevHubInternal*) hub;
    RhspModuleInterfaceListInternal* list = (RhspModuleInterfaceListInternal*) internalHub->interfaceList;
    //printf("antes do internal\n");
    uint16_t packet_id = getInterfacePacketIdInternal(list, interfaceName, functionNumber);
    //printf("depois do internal\n");
    if (packet_id != RHSP_INTERFACE_INVALID_PACKET_ID)
    {
        if (packetID)
        {
            *packetID = packet_id;
        }
        return RHSP_RESULT_OK;
    }
    //printf("antes do query\n");
    int retval = rhsp_queryInterface(hub, interfaceName, NULL, nackReasonCode);
    //printf("depois do query\n");
    if (retval < 0)
    {
        return retval;
    }

    list = (RhspModuleInterfaceListInternal*) internalHub->interfaceList; //query can update the list
    //printf("antes do internal 2\n");
    packet_id = getInterfacePacketIdInternal(list, interfaceName, functionNumber);
    //printf("depois do internal 2\n");
    if (packet_id == RHSP_INTERFACE_INVALID_PACKET_ID)
    {
        return RHSP_ERROR_COMMAND_NOT_SUPPORTED;
    }
    if (packetID)
    {
        *packetID = packet_id;
    }
    return RHSP_RESULT_OK;
}

int rhsp_queryInterface(RhspRevHub* hub,
                        const char* interfaceName,
                        RhspModuleInterface* intf,
                        uint8_t* nackReasonCode)
{
    size_t interfaceNameLength = strlen(interfaceName) + 1; // interfaceNameLength should include null terminated symbol

    rhsp_assert(interfaceNameLength <= RHSP_MAX_PAYLOAD_SIZE);

    if (!hub || interfaceNameLength > RHSP_MAX_PAYLOAD_SIZE)
    {
        return RHSP_ERROR;
    }

    int result = rhsp_sendReadCommandInternal(hub, 0x7F07, (const uint8_t*) interfaceName,
                                              (uint16_t) interfaceNameLength, nackReasonCode);
    if (result < 0)
    {
        return result;
    }

    RhspRevHubInternal* internalHub = (RhspRevHubInternal*) hub;
    // save discovered interface for further using in device control messages.
    RhspModuleInterface intf_;
    intf_.name = malloc(interfaceNameLength);
    memcpy(intf_.name, interfaceName, interfaceNameLength);
    intf_.name[interfaceNameLength-1] = 0;
    //printf("len %d\n", interfaceNameLength);
    //printf("da string isa %s\n", intf_.name);
    intf_.firstPacketID = RHSP_ARRAY_WORD(uint16_t, RHSP_PACKET_PAYLOAD_PTR(internalHub->rxBuffer), 0);
    intf_.numberIDValues = RHSP_ARRAY_WORD(uint16_t, RHSP_PACKET_PAYLOAD_PTR(internalHub->rxBuffer), 2);
    addInterface(internalHub, &intf_);
    if (intf)
    {
        *intf = intf_;
    }
    return RHSP_RESULT_OK;
}
